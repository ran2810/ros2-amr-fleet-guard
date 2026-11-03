#!/usr/bin/env python3
"""Per-robot agent (one per robot, namespaced /robot_N/robot_agent).

Owns everything about ONE robot that is not navigation:

  * mission FSM   IDLE -> NAVIGATING_TO_PICK -> PICK -> NAVIGATING_TO_DROP -> DROP -> IDLE
  * default position: after a failed mission, when idle for a while, or when the battery is low the
    agent drives the robot back to its dock (NAVIGATING_TO_DOCK); if even that fails -> FAULT
  * battery model (simulated state of charge): drains with distance driven and time, charges on the dock
  * neighbour awareness: the simulated LiDAR plane passes above the other robots' chassis, so every
    agent publishes the other robots' TF poses as filled discs of points (/robot_N/other_robots) that the
    Nav2 costmaps consume as an extra obstacle source

Interfaces (relative to the /robot_N namespace):
    action  execute_mission   warehouse_msgs/action/ExecuteMission   (from the fleet manager)
    topic   robot_state       warehouse_msgs/msg/RobotState           2 Hz
    topic   other_robots      sensor_msgs/PointCloud2 in 'map'        10 Hz
    service reset_fault       std_srvs/Trigger                        retry the dock return after a FAULT
    uses    navigate_to_pose  nav2_msgs/action/NavigateToPose, odom, TF (global /tf)

Battery numbers are deliberately fast so the behaviour shows up in minutes of simulation:
drain_per_meter / drain_idle / charge_rate are parameters.
"""
import math
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from warehouse_msgs.action import ExecuteMission
from warehouse_msgs.msg import RobotState as S

NAV_STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
STATE_NAMES = {0: 'IDLE', 1: 'NAVIGATING_TO_PICK', 2: 'PICK', 3: 'NAVIGATING_TO_DROP', 4: 'DROP',
               5: 'NAVIGATING_TO_DOCK', 6: 'CHARGE', 7: 'FAULT', 8: 'STARTING'}


def load_stations():
    path = get_package_share_directory('warehouse_gazebo') + '/config/stations.yaml'
    with open(path) as f:
        data = yaml.safe_load(f)
    return {s['id']: (float(s['x']), float(s['y']), float(s['yaw']))
            for key in ('docks', 'pick_stations', 'drop_stations') for s in data[key]}


class RobotAgent(Node):
    def __init__(self):
        super().__init__('robot_agent')
        p = self.declare_parameter
        self.robot_id = p('robot_id', 'robot_1').value
        self.dock_id = p('dock_id', 'dock_1').value
        self.robot_ids = list(p('robot_ids', ['robot_1']).value)
        # battery model (percent state of charge)
        self.initial_soc = float(p('initial_soc', 100.0).value)
        self.drain_per_meter = float(p('drain_per_meter', 0.25).value)    # % per metre driven
        self.drain_idle = float(p('drain_idle', 0.005).value)             # % per second
        self.charge_rate = float(p('charge_rate', 0.5).value)             # % per second on the dock
        self.min_mission_soc = float(p('min_mission_soc', 35.0).value)    # refuse missions below this
        self.low_soc = float(p('low_soc', 30.0).value)                    # idle + below -> go charge
        self.charge_resume_soc = float(p('charge_resume_soc', 90.0).value)  # on dock + below -> charge
        self.charge_until = float(p('charge_until', 95.0).value)
        # behaviour
        self.pick_time = float(p('pick_time', 3.0).value)
        self.drop_time = float(p('drop_time', 3.0).value)
        self.idle_return_after = float(p('idle_return_after', 15.0).value)  # s idle away from dock
        self.nav_retries = int(p('nav_retries', 2).value)
        self.dock_retries = int(p('dock_retries', 3).value)
        self.nav_timeout = float(p('nav_timeout', 300.0).value)
        self.dock_tolerance = float(p('dock_tolerance', 0.6).value)
        self.ring_radius = float(p('neighbour_ring_radius', 0.32).value)   # radius of the disc drawn per neighbour
        step = 0.05                                                         # = costmap resolution: no gaps
        n = int(self.ring_radius / step)
        self._disc = [(i * step, j * step) for i in range(-n, n + 1) for j in range(-n, n + 1)
                      if (i * step) ** 2 + (j * step) ** 2 <= self.ring_radius ** 2]

        self.stations = load_stations()
        self.dock = self.stations[self.dock_id]

        self.lock = threading.RLock()
        self.state = S.STARTING
        self.health = S.HEALTHY
        self.detail = ''
        self.task_id = ''
        self.soc = self.initial_soc
        self.pose = self.dock                      # (x, y, yaw) in map; refreshed from TF
        self.nearest = math.inf
        self.busy = False                          # a mission / dock-return worker owns the robot
        self.idle_since = None
        self._speed = 0.0
        self._last_tick = None
        self._cancel_requested = False
        self._near = False
        self._nav_handle = None

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self.state_pub = self.create_publisher(S, 'robot_state', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, 'other_robots', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=cb)
        self.create_service(Trigger, 'reset_fault', self._on_reset_fault, callback_group=cb)
        self.mission_server = ActionServer(
            self, ExecuteMission, 'execute_mission', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel, callback_group=cb)
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap', callback_group=cb),
            self.create_client(ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap', callback_group=cb)]
        self.ready_clients = [self.create_client(GetState, f'{n}/get_state', callback_group=cb)
                              for n in ('bt_navigator', 'amcl', 'velocity_smoother')]

        self.create_timer(0.5, self._publish_state, callback_group=cb)
        self.create_timer(0.2, self._tick, callback_group=cb)
        self.create_timer(0.1, self._publish_neighbours, callback_group=cb)
        threading.Thread(target=self._startup, daemon=True).start()
        self.get_logger().info(f'{self.robot_id}: agent up (dock {self.dock_id}); waiting for Nav2 ...')

    # ------------------------------------------------------------------ helpers
    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_state(self, state):
        with self.lock:
            if self.state != state:
                self.get_logger().info(
                    f'{self.robot_id}: {STATE_NAMES.get(self.state, self.state)} -> {STATE_NAMES.get(state, state)}')
            self.state = state

    def _finish_idle(self):
        """End of a worker: back to IDLE (stays FAULT if the robot was faulted meanwhile)."""
        with self.lock:
            self.state = S.FAULT if self.health == S.FAULTED else S.IDLE
            self.idle_since = self._now_s()
            self.task_id = ''
            self.busy = False

    def _lookup(self, robot_id):
        try:
            t = self.tf_buffer.lookup_transform('map', f'{robot_id}/base_link', Time())
        except Exception:   # TF not available (yet)
            return None
        q, tr = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tr.x, tr.y, yaw

    def _at_dock(self):
        return math.hypot(self.pose[0] - self.dock[0], self.pose[1] - self.dock[1]) <= self.dock_tolerance

    # ------------------------------------------------------------------ startup
    def _nav_ready(self):
        if not self.nav_client.server_is_ready():
            return False
        for client in self.ready_clients:
            if not client.service_is_ready():
                return False
            fut = client.call_async(GetState.Request())
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 2.0:
                time.sleep(0.05)
            if not fut.done() or fut.result().current_state.id != LifecycleState.PRIMARY_STATE_ACTIVE:
                return False
        return True

    def _startup(self):
        while rclpy.ok() and not self._nav_ready():
            time.sleep(2.0)
        with self.lock:
            self.state = S.IDLE
            self.idle_since = self._now_s()
        self.get_logger().info(f'{self.robot_id}: Nav2 active -> IDLE (battery {self.soc:.0f}%)')

    # ------------------------------------------------------------------ periodic work
    def _on_odom(self, msg):
        self._speed = abs(msg.twist.twist.linear.x)

    def _tick(self):
        """5 Hz: pose refresh, battery model, automatic dock return / charging."""
        pose = self._lookup(self.robot_id)
        now = self._now_s()
        with self.lock:
            if pose:
                self.pose = pose
            dt = 0.0 if self._last_tick is None else min(max(now - self._last_tick, 0.0), 1.0)
            self._last_tick = now
            self.soc -= self.drain_per_meter * self._speed * dt + self.drain_idle * dt
            if self.state == S.CHARGE:
                self.soc += self.charge_rate * dt
            self.soc = min(100.0, max(0.0, self.soc))

            if self.soc <= 0.0 and self.health != S.FAULTED:
                self.get_logger().error(f'{self.robot_id}: battery depleted -> FAULT')
                self.health, self.detail, self.state = S.FAULTED, 'battery depleted', S.FAULT
                self._cancel_requested = True       # stops any running navigation
                return
            if self.state == S.STARTING or self.busy:
                return
            if self.state == S.CHARGE:
                if self.soc >= self.charge_until:
                    self.get_logger().info(f'{self.robot_id}: charged to {self.soc:.0f}% -> IDLE')
                    self.state, self.idle_since = S.IDLE, now
                return
            if self.state != S.IDLE:
                return
            if self._at_dock():
                if self.soc < self.charge_resume_soc:
                    self.get_logger().info(f'{self.robot_id}: charging on {self.dock_id} (battery {self.soc:.0f}%)')
                    self.state = S.CHARGE
            elif self.soc < self.low_soc:
                self._start_dock_return(f'low battery ({self.soc:.0f}%)')
            elif self.idle_since is not None and now - self.idle_since >= self.idle_return_after:
                self._start_dock_return('idle: returning to default position')

    def _publish_state(self):
        msg = S()
        with self.lock:
            msg.stamp = self.get_clock().now().to_msg()
            msg.robot_id = self.robot_id
            msg.state, msg.health = self.state, self.health
            msg.x, msg.y, msg.yaw = (float(v) for v in self.pose)
            msg.battery = float(self.soc)
            msg.nearest_robot_m = float(self.nearest)
            msg.assigned_task = self.task_id
            msg.detail = self.detail
        self.state_pub.publish(msg)

    def _publish_neighbours(self):
        """Other robots' poses (from the global /tf) as filled discs of points the costmaps treat as obstacles."""
        points, nearest = [], math.inf
        for rid in self.robot_ids:
            if rid == self.robot_id:
                continue
            pose = self._lookup(rid)
            if pose is None:
                continue
            cx, cy, _ = pose
            nearest = min(nearest, math.hypot(cx - self.pose[0], cy - self.pose[1]))
            points += [(cx + dx, cy + dy, 0.1) for dx, dy in self._disc]
        if not points:   # a point under our own footprint: harmless (footprint cells are cleared)
            points = [(self.pose[0], self.pose[1], 0.1)]
        header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))   # always: no stale ghosts
        self.nearest = nearest
        if nearest < 0.5 and not self._near:
            self._near = True
            self.get_logger().warn(f'{self.robot_id}: NEAR-COLLISION, other robot {nearest:.2f} m away')
        elif nearest > 0.7:
            self._near = False

    # ------------------------------------------------------------------ navigation
    def _navigate_once(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
        goal.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        fut = self.nav_client.send_goal_async(goal)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 10.0:
                return 'NO_ACK'
            time.sleep(0.05)
        handle = fut.result()
        if not handle.accepted:
            return 'REJECTED'
        self._nav_handle = handle
        result = handle.get_result_async()
        while not result.done():
            if self._cancel_requested or time.monotonic() - t0 > self.nav_timeout:
                timed_out = not self._cancel_requested
                handle.cancel_goal_async()
                t1 = time.monotonic()
                while not result.done() and time.monotonic() - t1 < 5.0:
                    time.sleep(0.05)
                return 'TIMEOUT' if timed_out else 'CANCELED'
            time.sleep(0.1)
        return NAV_STATUS.get(result.result().status, str(result.result().status))

    def _clear_costmaps(self):
        for client in self.clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    def _navigate(self, pose, retries=None):
        """Navigate with retries; returns True on success."""
        retries = self.nav_retries if retries is None else retries
        for attempt in range(retries + 1):
            if self._cancel_requested:
                return False
            outcome = self._navigate_once(pose)
            if outcome == 'SUCCEEDED':
                return True
            self.get_logger().warn(f'{self.robot_id}: navigation to ({pose[0]:.1f}, {pose[1]:.1f}) '
                                   f'{outcome} (attempt {attempt + 1}/{retries + 1})')
            if outcome == 'CANCELED' or self._cancel_requested:
                return False
            self._clear_costmaps()          # stale obstacles (e.g. a robot that has left) are a common cause
            time.sleep(3.0)
        return False

    def _dwell(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds and not self._cancel_requested:
            time.sleep(0.1)

    # ------------------------------------------------------------------ dock return / faults
    def _start_dock_return(self, reason):
        """Caller holds the lock and has checked that the robot is free."""
        self.busy = True
        self._cancel_requested = False
        threading.Thread(target=self._dock_return, args=(reason,), daemon=True).start()

    def _dock_return(self, reason):
        self.get_logger().info(f'{self.robot_id}: returning to {self.dock_id}: {reason}')
        with self.lock:
            self.state = S.NAVIGATING_TO_DOCK
            if self.health == S.HEALTHY:
                self.detail = reason
        ok = self._navigate(self.dock, retries=self.dock_retries)
        with self.lock:
            if ok:
                self.health, self.detail = S.HEALTHY, ''
            elif self.health != S.FAULTED:      # (a depleted battery already faulted the robot)
                self.health, self.detail = S.FAULTED, 'could not return to dock'
                self.get_logger().error(f'{self.robot_id}: dock return failed -> FAULT')
        self._finish_idle()

    def _on_reset_fault(self, _request, response):
        with self.lock:
            if self.state == S.FAULT and not self.busy and self.soc > 0.0:
                self.health, self.detail = S.DEGRADED, 'fault reset'
                self._start_dock_return('fault reset')
                response.success, response.message = True, 'returning to dock'
            else:
                response.success, response.message = False, 'not in a resettable FAULT'
        return response

    # ------------------------------------------------------------------ mission action
    def _on_goal(self, request):
        with self.lock:
            reason = None
            if self.busy:
                reason = 'busy with another mission or returning to the dock'
            elif self.state not in (S.IDLE, S.CHARGE):
                reason = f'not available (state {STATE_NAMES.get(self.state, self.state)})'
            elif self.health != S.HEALTHY:
                reason = 'not healthy'
            elif self.soc < self.min_mission_soc:
                reason = f'battery {self.soc:.0f}% below {self.min_mission_soc:.0f}%'
            elif request.pickup_id not in self.stations or request.drop_id not in self.stations:
                reason = 'unknown station'
            if reason:
                self.get_logger().warn(f'{self.robot_id}: rejecting task {request.task_id}: {reason}')
                return GoalResponse.REJECT
            self.busy = True
            self._cancel_requested = False
            return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    def _execute(self, gh):
        req = gh.request
        result = ExecuteMission.Result()
        with self.lock:
            self.task_id = f'{req.task_id} {req.pickup_id}>{req.drop_id}'   # shown in RobotState.assigned_task
        self.get_logger().info(f'{self.robot_id}: task {req.task_id}: {req.pickup_id} -> {req.drop_id} '
                               f'(battery {self.soc:.0f}%)')
        steps = [(S.NAVIGATING_TO_PICK, 'navigate_to_pickup', self.stations[req.pickup_id]),
                 (S.PICK, 'pick', self.pick_time),
                 (S.NAVIGATING_TO_DROP, 'navigate_to_drop', self.stations[req.drop_id]),
                 (S.DROP, 'drop', self.drop_time)]
        failure = None
        for state, stage, arg in steps:
            if self._cancel_requested:
                break
            self._set_state(state)
            fb = ExecuteMission.Feedback()
            fb.stage, fb.battery = stage, float(self.soc)
            gh.publish_feedback(fb)
            if isinstance(arg, tuple):
                if not self._navigate(arg):
                    failure = f'{stage} failed'
                    break
            else:
                self._dwell(arg)

        if failure and not self._cancel_requested:
            with self.lock:
                self.health, self.detail, self.task_id = S.DEGRADED, failure, ''
            result.success, result.message = False, failure
            gh.abort()
            with self.lock:   # busy stays True: the dock-return worker owns the robot now
                self._start_dock_return(f'after failed task {req.task_id}')
            return result
        if self._cancel_requested:
            result.success, result.message = False, 'canceled'
            gh.canceled()
        else:
            result.success, result.message = True, 'done'
            gh.succeed()
            self.get_logger().info(f'{self.robot_id}: task {req.task_id} done (battery {self.soc:.0f}%)')
        self._finish_idle()
        return result


def main():
    rclpy.init()
    node = RobotAgent()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Fleet manager: a task queue and a deliberately simple, explainable allocation rule.

    task arrives -> keep robots that are fresh, IDLE/CHARGE, HEALTHY, battery >= min_battery
                 -> exclude robots that already failed this task
                 -> pick the one closest to the pickup station (straight-line distance)
                 -> send ExecuteMission; on failure re-queue (max_attempts), on success count it

It talks to the per-robot agents only through warehouse_msgs; it never touches Nav2 directly, and the
robots' autonomy (navigation, battery, return to dock) stays in the agents.

    ros2 service call /fleet/submit_task warehouse_msgs/srv/SubmitTask "{pickup_id: pick_1, drop_id: drop_3}"
    ros2 topic echo /fleet/status
"""
import math
from collections import deque
from dataclasses import dataclass, field

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from warehouse_msgs.action import ExecuteMission
from warehouse_msgs.msg import FleetStatus
from warehouse_msgs.msg import RobotState as S
from warehouse_msgs.srv import RequestDock, SubmitTask

# one colour per robot (same order as the robot models in RViz)
PALETTE = [(0.15, 0.45, 0.85), (0.85, 0.20, 0.20), (0.20, 0.70, 0.30), (0.90, 0.75, 0.10),
           (0.60, 0.25, 0.80), (0.10, 0.75, 0.80), (0.95, 0.45, 0.10), (0.90, 0.40, 0.65)]

STATE_NAMES = {0: 'IDLE', 1: 'NAVIGATING_TO_PICK', 2: 'PICK', 3: 'NAVIGATING_TO_DROP', 4: 'DROP',
               5: 'NAVIGATING_TO_DOCK', 6: 'CHARGE', 7: 'FAULT', 8: 'STARTING'}


def load_stations():
    """(pick/drop stations {id: (x, y)}, docks {id: (x, y)}) from the generated layout."""
    path = get_package_share_directory('warehouse_gazebo') + '/config/stations.yaml'
    with open(path) as f:
        data = yaml.safe_load(f)
    stations = {s['id']: (float(s['x']), float(s['y'])) for key in ('pick_stations', 'drop_stations') for s in data[key]}
    docks = {s['id']: (float(s['x']), float(s['y'])) for s in data['docks']}
    return stations, docks


@dataclass
class Task:
    task_id: str
    pickup_id: str
    drop_id: str
    submitted: float
    attempts: int = 0
    excluded: set = field(default_factory=set)
    robot: str = ''
    started: float = 0.0
    handle: object = None
    blocked_by: str = ''            # last logged reason it is waiting for a station


@dataclass
class Robot:
    robot_id: str
    client: object
    state: object = None            # last RobotState
    last_update: float = 0.0
    busy: bool = False              # a mission goal is in flight to this robot
    cooldown_until: float = 0.0


class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        p = self.declare_parameter
        robot_ids = list(p('robot_ids', ['robot_1', 'robot_2']).value)
        self.min_battery = float(p('min_battery', 40.0).value)          # do not assign below this SOC [%]
        self.stale_after = float(p('stale_after', 5.0).value)           # s without a state message
        self.max_attempts = int(p('max_attempts', 3).value)
        self.assignment_timeout = float(p('assignment_timeout', 900.0).value)
        # must match the agents' energy estimate (same launch arguments)
        self.detour_factor = float(p('detour_factor', 1.5).value)
        self.reserve_soc = float(p('reserve_soc', 10.0).value)
        self.dock_clearance = float(p('dock_clearance', 0.7).value)   # a robot this close occupies a dock
        self.stations, self.docks = load_stations()
        self.dock_holder = {}            # dock id -> robot id it is reserved for

        self.robots = {}
        for rid in robot_ids:
            client = ActionClient(self, ExecuteMission, f'/{rid}/execute_mission')
            self.robots[rid] = Robot(rid, client)
            self.create_subscription(S, f'/{rid}/robot_state', lambda msg, r=rid: self._on_state(r, msg), 10)

        self.queue = deque()
        self.active = {}                 # task_id -> Task
        self.done = self.failed = 0
        self.task_counter = 0
        self.create_service(SubmitTask, '/fleet/submit_task', self._on_submit)
        self.create_service(RequestDock, '/fleet/request_dock', self._on_request_dock)
        self.status_pub = self.create_publisher(FleetStatus, '/fleet/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        self.create_timer(1.0, self._allocate)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f'fleet manager up: robots {robot_ids}, min battery {self.min_battery:.0f}%')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def _on_state(self, rid, msg):
        r = self.robots[rid]
        r.state, r.last_update = msg, self._now()
        if msg.state in (S.NAVIGATING_TO_PICK, S.PICK, S.NAVIGATING_TO_DROP, S.DROP):
            self._release_docks(rid)          # it has left its dock: the dock is free for others

    def _on_submit(self, request, response):
        if request.pickup_id not in self.stations or request.drop_id not in self.stations:
            response.accepted, response.message = False, 'unknown station id'
            return response
        self.task_counter += 1
        task = Task(f'task_{self.task_counter}', request.pickup_id, request.drop_id, self._now())
        self.queue.append(task)
        response.accepted, response.task_id = True, task.task_id
        response.message = f'queued ({len(self.queue)} waiting)'
        self.get_logger().info(f'{task.task_id} queued: {task.pickup_id} -> {task.drop_id}')
        return response

    # ------------------------------------------------------------------ docks
    def _release_docks(self, rid):
        for dock_id in [d for d, who in self.dock_holder.items() if who == rid]:
            del self.dock_holder[dock_id]

    def _on_request_dock(self, request, response):
        """Nearest dock that is not reserved for another robot and has no other robot sitting on it."""
        rid = request.robot_id
        occupied = set()
        for other in self.robots.values():
            if other.robot_id == rid or other.state is None:
                continue
            for dock_id, (dx, dy) in self.docks.items():
                if math.hypot(other.state.x - dx, other.state.y - dy) < self.dock_clearance:
                    occupied.add(dock_id)
        candidates = [(math.hypot(request.x - dx, request.y - dy), dock_id)
                      for dock_id, (dx, dy) in self.docks.items()
                      if dock_id not in occupied and dock_id not in request.exclude
                      and self.dock_holder.get(dock_id, rid) == rid]
        if not candidates:
            response.granted, response.message = False, 'no free dock'
            self.get_logger().warn(f'{rid} asked for a dock: none free')
            return response
        distance, dock_id = min(candidates)
        self._release_docks(rid)
        self.dock_holder[dock_id] = rid
        response.granted, response.dock_id = True, dock_id
        response.message = f'{distance:.1f} m away'
        self.get_logger().info(f'{rid} -> {dock_id} ({distance:.1f} m away)')
        return response

    # ------------------------------------------------------------------ station locks
    def _reserved_stations(self):
        """station id -> task id holding it. A task holds its drop station from the moment it is
        assigned until it finishes, and its pickup station until the robot has driven away from it.
        Two robots are never sent to the same station at the same time."""
        held = {}
        for t in self.active.values():
            r = self.robots.get(t.robot)
            left_pickup = (r is not None and r.state is not None
                           and r.state.state in (S.NAVIGATING_TO_DROP, S.DROP))
            if not left_pickup:
                held[t.pickup_id] = t.task_id
            held[t.drop_id] = t.task_id
        return held

    # ------------------------------------------------------------------ allocation
    def _energy_needed(self, s, task):
        """Percent of battery for: robot -> pickup -> drop -> nearest dock (+ reserve), same rule as the agent."""
        px, py = self.stations[task.pickup_id]
        dx, dy = self.stations[task.drop_id]
        to_dock = min(math.hypot(dx - x, dy - y) for x, y in self.docks.values())
        metres = math.hypot(px - s.x, py - s.y) + math.hypot(dx - px, dy - py) + to_dock
        return s.drain_per_meter * self.detour_factor * metres + self.reserve_soc

    def _eligible(self, r, task, now):
        s = r.state
        return (s is not None and not r.busy and r.robot_id not in task.excluded
                and now - r.last_update <= self.stale_after and now >= r.cooldown_until
                and s.state in (S.IDLE, S.CHARGE) and s.health == S.HEALTHY
                and s.battery >= max(self.min_battery, self._energy_needed(s, task)))

    def _allocate(self):
        now = self._now()
        for task in list(self.queue):
            held = self._reserved_stations()
            busy = [(sid, held[sid]) for sid in (task.pickup_id, task.drop_id) if sid in held]
            if busy:       # wait for the station; later tasks in the queue may go first
                reason = ', '.join(f'{sid} (held by {tid})' for sid, tid in busy)
                if task.blocked_by != reason:
                    task.blocked_by = reason
                    self.get_logger().info(f'{task.task_id} waits for station {reason}')
                continue
            task.blocked_by = ''
            px, py = self.stations[task.pickup_id]
            candidates = [(math.hypot(r.state.x - px, r.state.y - py), r.robot_id)
                          for r in self.robots.values() if self._eligible(r, task, now)]
            if not candidates:
                continue
            distance, rid = min(candidates)
            self.queue.remove(task)
            self._assign(task, rid, distance)
        # watchdog for missions that never come back
        for task in list(self.active.values()):
            if task.handle is not None and now - task.started > self.assignment_timeout:
                self.get_logger().error(f'{task.task_id}: timed out on {task.robot}, canceling')
                task.handle.cancel_goal_async()

    def _assign(self, task, rid, distance):
        r = self.robots[rid]
        if not r.client.server_is_ready():
            self.queue.appendleft(task)
            r.cooldown_until = self._now() + 5.0
            return
        r.busy = True
        task.robot, task.started = rid, self._now()
        task.attempts += 1
        self.active[task.task_id] = task
        goal = ExecuteMission.Goal(task_id=task.task_id, pickup_id=task.pickup_id, drop_id=task.drop_id)
        self.get_logger().info(
            f'{task.task_id} -> {rid} (pickup {distance:.1f} m away, battery {r.state.battery:.0f}%, '
            f'attempt {task.attempts}/{self.max_attempts})')
        r.client.send_goal_async(goal).add_done_callback(lambda fut, t=task: self._on_goal_response(t, fut))

    # ------------------------------------------------------------------ results
    def _on_goal_response(self, task, fut):
        r = self.robots[task.robot]
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().warn(f'{task.task_id}: {task.robot} rejected it, re-queued')
            r.busy, r.cooldown_until = False, self._now() + 5.0
            task.attempts -= 1                      # a rejection is not a failed attempt
            self.active.pop(task.task_id, None)
            self.queue.appendleft(task)
            return
        task.handle = handle
        handle.get_result_async().add_done_callback(lambda f, t=task: self._on_result(t, f))

    def _on_result(self, task, fut):
        r = self.robots[task.robot]
        r.busy = False
        self.active.pop(task.task_id, None)
        wrapped = fut.result()
        ok = wrapped.status == 4 and wrapped.result.success
        duration = self._now() - task.started
        if ok:
            self.done += 1
            self.get_logger().info(f'{task.task_id} DONE by {task.robot} in {duration:.0f} s')
            return
        message = wrapped.result.message
        if message.startswith('low battery') or message == 'battery depleted':
            task.attempts -= 1                       # not the task's fault: another robot gets a fresh try
            task.handle = None
            self.get_logger().warn(f'{task.task_id} dropped by {task.robot} ({message}); re-queued')
            self.queue.appendleft(task)
            return
        task.excluded.add(task.robot)
        if task.attempts >= self.max_attempts:
            self.failed += 1
            self.get_logger().error(f'{task.task_id} FAILED for good after {task.attempts} attempts ({message})')
        else:
            self.get_logger().warn(f'{task.task_id} failed on {task.robot} ({message}); re-queued')
            task.handle = None
            self.queue.appendleft(task)

    # ------------------------------------------------------------------ status
    def _publish_status(self):
        msg = FleetStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.robots = [r.state for r in self.robots.values() if r.state is not None]
        msg.tasks_queued, msg.tasks_active = len(self.queue), len(self.active)
        msg.tasks_done, msg.tasks_failed = self.done, self.failed
        self.status_pub.publish(msg)
        self._publish_markers()

    def _marker(self, ns, mid, mtype, x, y, z, scale, rgba, text=''):
        m = Marker()
        m.header.frame_id, m.ns, m.id, m.type, m.action = 'map', ns, mid, mtype, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.text = text
        return m

    def _publish_markers(self):
        """RViz view of the fleet's work: stations, each active task as a route in its robot's colour,
        and the waiting queue. Topic /fleet/markers (add a MarkerArray display, fixed frame 'map')."""
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        held = self._reserved_stations()
        for i, (sid, (x, y)) in enumerate(self.stations.items()):
            colour = (0.15, 0.40, 0.85, 1.0) if sid.startswith('pick') else (0.95, 0.50, 0.10, 1.0)
            if sid in held:   # reserved: red disc, label shows the holder
                colour = (0.90, 0.10, 0.10, 1.0)
            arr.markers.append(self._marker('stations', i, Marker.CYLINDER, x, y, 0.05, (0.35, 0.35, 0.05), colour))
            arr.markers.append(self._marker('station_names', i, Marker.TEXT_VIEW_FACING, x, y, 0.7,
                                            (0.0, 0.0, 0.3), (1.0, 1.0, 1.0, 1.0),
                                            f'{sid} [{held[sid]}]' if sid in held else sid))
        for i, (dock_id, (x, y)) in enumerate(self.docks.items()):
            holder = self.dock_holder.get(dock_id)
            colour = (0.95, 0.80, 0.10, 1.0) if holder else (0.10, 0.70, 0.30, 1.0)
            arr.markers.append(self._marker('docks', i, Marker.CYLINDER, x, y, 0.03, (0.45, 0.45, 0.03), colour))
            if holder:
                arr.markers.append(self._marker('dock_names', i, Marker.TEXT_VIEW_FACING, x, y, 0.5,
                                                (0.0, 0.0, 0.25), (1.0, 1.0, 0.3, 1.0), f'{dock_id} [{holder}]'))
        for i, rec in enumerate(self.robots.values()):   # robot id in the colour of its model
            if rec.state is not None:
                red, green, blue = PALETTE[(int(rec.robot_id.split('_')[-1]) - 1) % len(PALETTE)]
                arr.markers.append(self._marker('robot_names', i, Marker.TEXT_VIEW_FACING, rec.state.x, rec.state.y,
                                                0.55, (0.0, 0.0, 0.28), (red, green, blue, 1.0), rec.robot_id))
        for i, task in enumerate(self.active.values()):
            r = self.robots.get(task.robot)
            if r is None or r.state is None:
                continue
            red, green, blue = PALETTE[(int(task.robot.split('_')[-1]) - 1) % len(PALETTE)]
            px, py = self.stations[task.pickup_id]
            dx, dy = self.stations[task.drop_id]
            where = [(r.state.x, r.state.y)]
            if r.state.state in (S.IDLE, S.CHARGE, S.NAVIGATING_TO_PICK):
                where += [(px, py), (dx, dy)]          # still has to go to the pickup, then the drop
            elif r.state.state in (S.PICK, S.NAVIGATING_TO_DROP):
                where += [(dx, dy)]
            line = self._marker('task_routes', i, Marker.LINE_STRIP, 0, 0, 0, (0.07, 0.0, 0.0), (red, green, blue, 0.9))
            line.points = [Point(x=float(x), y=float(y), z=0.15) for x, y in where]
            arr.markers.append(line)
            arr.markers.append(self._marker('task_labels', i, Marker.TEXT_VIEW_FACING, r.state.x, r.state.y, 0.9,
                                            (0.0, 0.0, 0.3), (red, green, blue, 1.0),
                                            f'{task.task_id}: {task.pickup_id}->{task.drop_id}'))
        waiting = '  '.join(f'{t.task_id}:{t.pickup_id}->{t.drop_id}' for t in list(self.queue)[:5])
        more = f' (+{len(self.queue) - 5} more)' if len(self.queue) > 5 else ''
        summary = (f'queue {len(self.queue)}: {waiting or "empty"}{more}\n'
                   f'active {len(self.active)}  done {self.done}  failed {self.failed}')
        arr.markers.append(self._marker('queue', 0, Marker.TEXT_VIEW_FACING, -11.0, 7.0, 0.1,
                                        (0.0, 0.0, 0.35), (1.0, 1.0, 1.0, 1.0), summary))
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = FleetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

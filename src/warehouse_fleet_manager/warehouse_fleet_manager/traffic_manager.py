#!/usr/bin/env python3
"""Fleet traffic manager for the warehouse AMRs.

The task allocator remains in manager.py. Robot agents ask this node to move one leg (to pickup,
drop, or dock). The traffic manager combines rolling reservations with direction-aware aisle
ownership:

  * ordinary lanes / intersections remain exclusive resources,
  * rack aisles may be shared by a limited number of robots travelling in the same direction,
  * opposite-direction aisle entry is rejected while that aisle is owned,
  * same-direction followers wait for minimum spacing instead of immediately rerouting,
  * contiguous graph edges belonging to one physical aisle are traversed as one Nav2 segment,
  * rolling look-ahead is still acquired before resources behind the robot are released.

This targets the dominant head-on collision case without serialising the complete warehouse. The
local costmap / other-robot obstacle layer remains the final physical separation mechanism between
same-direction followers.
"""
from __future__ import annotations

import heapq
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from warehouse_msgs.action import TrafficNavigate
from warehouse_msgs.msg import RobotState

Point = Tuple[float, float]


@dataclass(frozen=True)
class Edge:
    edge_id: str
    src: str
    dst: str
    resource: str
    length: float


@dataclass
class RoutePlan:
    nodes: List[str]
    edges: List[Edge]
    cost: float


@dataclass
class Lease:
    robot_id: str
    route_id: str
    acquired_at: float
    direction: str = ''


@dataclass(frozen=True)
class ResourceRequest:
    resource: str
    direction: str = ''


class TrafficGraph:
    def __init__(self, path: str):
        with open(path) as f:
            data = yaml.safe_load(f)
        self.frame_id = data.get('frame_id', 'map')
        routing = data.get('routing', {})
        self.waypoint_spacing = float(routing.get('waypoint_spacing', 0.6))
        self.nearest_nodes = int(routing.get('nearest_nodes', 1))
        self.anchor_snap_dist = float(routing.get('anchor_snap_dist', 0.75))
        self.lock_route_nodes = bool(routing.get('lock_route_nodes', True))
        self.nodes: Dict[str, Point] = {
            str(name): (float(v['x']), float(v['y'])) for name, v in data['nodes'].items()
        }
        resource_data = data.get('resources') or {}
        self.resources: Dict[str, Set[str]] = {
            str(name): set(v.get('conflicts_with', []) or [])
            for name, v in resource_data.items()
        }
        self.directional_resources: Set[str] = {
            str(name) for name, v in resource_data.items() if bool(v.get('directional', False))
        }
        self.resource_capacity: Dict[str, int] = {
            str(name): max(1, int(v.get('capacity', 1))) for name, v in resource_data.items()
        }
        self.directional_min_spacing = float(routing.get('directional_min_spacing', 1.5))
        self.adj: Dict[str, List[Edge]] = {name: [] for name in self.nodes}
        for raw in data['edges']:
            src, dst = str(raw['from']), str(raw['to'])
            resource = str(raw.get('resource') or '')
            length = math.dist(self.nodes[src], self.nodes[dst])
            edge = Edge(str(raw['id']), src, dst, resource, length)
            self.adj[src].append(edge)
            if bool(raw.get('bidirectional', True)):
                self.adj[dst].append(Edge(edge.edge_id, dst, src, resource, length))

    @staticmethod
    def _d(a: Point, b: Point) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def edge_direction(self, edge: Edge) -> str:
        """Cardinal travel direction for a graph edge."""
        ax, ay = self.nodes[edge.src]
        bx, by = self.nodes[edge.dst]
        dx, dy = bx - ax, by - ay
        if abs(dx) >= abs(dy):
            return 'E' if dx >= 0.0 else 'W'
        return 'N' if dy >= 0.0 else 'S'

    def edge_request(self, edge: Edge) -> ResourceRequest:
        direction = self.edge_direction(edge) if edge.resource in self.directional_resources else ''
        return ResourceRequest(edge.resource, direction)

    def _nearest(self, p: Point) -> List[str]:
        ranked = sorted((self._d(p, xy), name) for name, xy in self.nodes.items())
        if ranked and ranked[0][0] <= self.anchor_snap_dist:
            return [ranked[0][1]]
        return [name for _, name in ranked[:max(1, self.nearest_nodes)]]

    def _astar(self, start: str, goal: str, blocked: Set[str]) -> Optional[RoutePlan]:
        if start == goal:
            return RoutePlan([start], [], 0.0)
        queue = [(0.0, 0.0, start)]
        came: Dict[str, Tuple[str, Edge]] = {}
        best = {start: 0.0}
        while queue:
            _, g, node = heapq.heappop(queue)
            if g > best.get(node, math.inf) + 1e-9:
                continue
            if node == goal:
                nodes = [goal]
                edges: List[Edge] = []
                cur = goal
                while cur != start:
                    prev, edge = came[cur]
                    edges.append(edge)
                    nodes.append(prev)
                    cur = prev
                nodes.reverse()
                edges.reverse()
                return RoutePlan(nodes, edges, g)

            for edge in self.adj[node]:
                if edge.resource and edge.resource in blocked:
                    continue
                if self.lock_route_nodes and edge.dst != goal and f'node:{edge.dst}' in blocked:
                    continue
                ng = g + edge.length
                if ng + 1e-9 >= best.get(edge.dst, math.inf):
                    continue
                best[edge.dst] = ng
                came[edge.dst] = (node, edge)
                h = self._d(self.nodes[edge.dst], self.nodes[goal])
                heapq.heappush(queue, (ng + h, ng, edge.dst))
        return None

    def plan(self, start_xy: Point, goal_xy: Point, blocked: Set[str]) -> Optional[RoutePlan]:
        """Best graph route including off-graph connector distance at each end."""
        best_plan = None
        best_total = math.inf
        for s in self._nearest(start_xy):
            for g in self._nearest(goal_xy):
                plan = self._astar(s, g, blocked)
                if plan is None:
                    continue
                total = self._d(start_xy, self.nodes[s]) + plan.cost + self._d(self.nodes[g], goal_xy)
                if total < best_total:
                    best_total = total
                    best_plan = plan
        if best_plan is not None:
            best_plan.cost = best_total
        return best_plan

    def route_resources(self, plan: RoutePlan) -> List[str]:
        """All resources used by a complete route (mainly useful for diagnostics)."""
        out: List[str] = []
        seen: Set[str] = set()
        for edge in plan.edges:
            if edge.resource and edge.resource not in seen:
                out.append(edge.resource)
                seen.add(edge.resource)
        if self.lock_route_nodes:
            for node in plan.nodes[1:-1]:
                resource = f'node:{node}'
                if resource not in seen:
                    out.append(resource)
                    seen.add(resource)
        return out

    def window_requests(
        self,
        plan: RoutePlan,
        start_edge: int = 0,
        lookahead_resources: int = 2,
    ) -> List[ResourceRequest]:
        """Rolling reservation requests, including direction for direction-owned aisles.

        The look-ahead is counted in distinct edge resources. Internal graph nodes inside one
        physical direction-owned aisle are not locked separately; otherwise they would serialize
        same-direction followers even though the aisle itself allows shared ownership.
        """
        if start_edge >= len(plan.edges):
            return []

        out: List[ResourceRequest] = []
        seen: Set[str] = set()
        edge_stop = start_edge
        distinct_edge_resources = 0

        for idx in range(start_edge, len(plan.edges)):
            edge = plan.edges[idx]
            if edge.resource and edge.resource not in seen:
                out.append(self.edge_request(edge))
                seen.add(edge.resource)
                distinct_edge_resources += 1
            edge_stop = idx + 1
            if distinct_edge_resources >= max(1, lookahead_resources):
                break

        if self.lock_route_nodes:
            for node_index in range(start_edge + 1, edge_stop + 1):
                if node_index >= len(plan.nodes) - 1:
                    continue

                prev_edge = plan.edges[node_index - 1]
                next_edge = plan.edges[node_index] if node_index < len(plan.edges) else None
                # A node between two halves of the same physical directional aisle is not an
                # intersection resource; the aisle ownership already protects it.
                if (
                    prev_edge.resource in self.directional_resources
                    and next_edge is not None
                    and next_edge.resource == prev_edge.resource
                ):
                    continue

                resource = f'node:{plan.nodes[node_index]}'
                if resource not in seen:
                    out.append(ResourceRequest(resource))
                    seen.add(resource)
        return out

    def follower_requests(self, plan: RoutePlan) -> List[ResourceRequest]:
        """Minimal admission window for a same-direction follower entering an owned aisle.

        The leader already owns the aisle direction and normally owns the exit look-ahead. Requiring
        that same exit resource for the follower would make direction-aware ownership ineffective.
        The follower therefore shares only the aisle resource and relies on rolling acquisition at
        the aisle exit. It may queue behind the leader, but no opposite-direction robot can enter.
        """
        if not plan.edges:
            return []
        first = self.edge_request(plan.edges[0])
        if first.resource not in self.directional_resources:
            return self.window_requests(plan, lookahead_resources=1)
        return [first]

    def chunk_edge_count(self, plan: RoutePlan) -> int:
        """Traverse a complete physical direction-owned aisle without stopping at its mid-node."""
        if not plan.edges:
            return 0
        first_resource = plan.edges[0].resource
        if first_resource not in self.directional_resources:
            return 1
        count = 1
        while count < len(plan.edges) and plan.edges[count].resource == first_resource:
            count += 1
        return count

    @staticmethod
    def format_requests(requests: Sequence[ResourceRequest]) -> List[str]:
        return [f'{r.resource}:{r.direction}' if r.direction else r.resource for r in requests]

    def _densify(
        self,
        raw: Sequence[Point],
        final_yaw: float,
    ) -> List[Tuple[float, float, float]]:
        compact = [raw[0]]
        for p in raw[1:]:
            if self._d(compact[-1], p) > 0.05:
                compact.append(p)
        if len(compact) == 1:
            return [(compact[0][0], compact[0][1], final_yaw)]

        points: List[Point] = []
        for a, b in zip(compact, compact[1:]):
            dist = self._d(a, b)
            steps = max(1, int(math.ceil(dist / max(0.2, self.waypoint_spacing))))
            for i in range(1, steps + 1):
                t = i / steps
                points.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))

        result: List[Tuple[float, float, float]] = []
        for i, (x, y) in enumerate(points):
            if i == len(points) - 1:
                yaw = final_yaw
            else:
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            result.append((x, y, yaw))
        return result

    def dense_points(
        self,
        start_xy: Point,
        goal: Tuple[float, float, float],
        plan: RoutePlan,
    ) -> List[Tuple[float, float, float]]:
        """Dense via-points for a complete graph route."""
        raw: List[Point] = [start_xy]
        raw.extend(self.nodes[n] for n in plan.nodes)
        raw.append((goal[0], goal[1]))
        return self._densify(raw, goal[2])

    def dense_segment_points(
        self,
        start_xy: Point,
        target: Tuple[float, float, float],
    ) -> List[Tuple[float, float, float]]:
        """Dense points for one already-approved graph edge.

        The current physical pose is connected directly to the edge destination. We deliberately do
        not force the robot back through the source graph node, which avoids small backtracking peaks
        after Nav2 reports success near (but not exactly on) a node.
        """
        return self._densify([start_xy, (target[0], target[1])], target[2])


class ReservationTable:
    def __init__(
        self,
        conflicts: Dict[str, Set[str]],
        directional_resources: Set[str],
        capacities: Dict[str, int],
        logger,
    ):
        self.conflicts = conflicts
        self.directional_resources = directional_resources
        self.capacities = capacities
        self.logger = logger
        self.lock = threading.RLock()
        # resource -> robot_id -> lease. Directional resources may have multiple same-direction
        # owners; ordinary resources still effectively have capacity one.
        self.owners: Dict[str, Dict[str, Lease]] = {}
        self.wait_for: Dict[str, Set[str]] = {}
        self.wait_since: Dict[str, float] = {}
        self.yield_until: Dict[str, float] = {}

    def _resource_leases_unlocked(self, resource: str) -> Dict[str, Lease]:
        return self.owners.get(resource, {})

    def _blockers_unlocked(self, request: ResourceRequest, robot_id: str) -> Set[str]:
        resource = request.resource
        blockers: Set[str] = set()
        leases = self._resource_leases_unlocked(resource)
        others = {rid: lease for rid, lease in leases.items() if rid != robot_id}

        if resource in self.directional_resources:
            # Directional ownership: same-direction sharing up to configured capacity. Any opposite
            # direction lease blocks admission immediately.
            opposite = {
                rid for rid, lease in others.items()
                if not request.direction or lease.direction != request.direction
            }
            blockers |= opposite
            if not opposite:
                capacity = self.capacities.get(resource, 1)
                same = {
                    rid for rid, lease in others.items()
                    if lease.direction == request.direction
                }
                if len(same) >= capacity:
                    blockers |= same
        else:
            blockers |= set(others)

        # Conflicting resources (e.g. horizontal vs centre vertical aisle) always block each other,
        # irrespective of travel direction.
        for other_resource in self.conflicts.get(resource, set()):
            blockers |= {
                rid for rid in self._resource_leases_unlocked(other_resource)
                if rid != robot_id
            }
        return blockers

    def blockers(self, requests: Iterable[ResourceRequest], robot_id: str) -> Set[str]:
        with self.lock:
            out: Set[str] = set()
            for request in requests:
                out |= self._blockers_unlocked(request, robot_id)
            return out

    def blocked_resources(self, robot_id: str) -> Set[str]:
        """Conservative blocked set used only for alternate A* routing.

        Primary-route admission is direction-aware. Alternate routing intentionally treats any
        currently owned resource as blocked so it never creates a speculative opposite-direction
        route through an active aisle.
        """
        with self.lock:
            blocked: Set[str] = set()
            for resource, leases in self.owners.items():
                if any(rid != robot_id for rid in leases):
                    blocked.add(resource)
                    blocked.update(self.conflicts.get(resource, set()))
            return blocked

    def try_set_window(
        self,
        requests: Sequence[ResourceRequest],
        robot_id: str,
        route_id: str,
    ) -> Tuple[bool, Set[str]]:
        """Atomically acquire a new rolling window before releasing the old one."""
        desired: List[ResourceRequest] = []
        by_resource: Dict[str, ResourceRequest] = {}
        for request in requests:
            by_resource[request.resource] = request
        desired = list(by_resource.values())
        desired_set = set(by_resource)

        with self.lock:
            blockers: Set[str] = set()
            for request in desired:
                blockers |= self._blockers_unlocked(request, robot_id)
            if blockers:
                return False, blockers

            now = time.monotonic()
            for request in desired:
                bucket = self.owners.setdefault(request.resource, {})
                lease = bucket.get(robot_id)
                if (
                    lease is None
                    or lease.route_id != route_id
                    or lease.direction != request.direction
                ):
                    bucket[robot_id] = Lease(
                        robot_id=robot_id,
                        route_id=route_id,
                        acquired_at=now,
                        direction=request.direction,
                    )

            # Release this route's resources that are no longer part of the rolling window.
            for resource in list(self.owners):
                bucket = self.owners[resource]
                lease = bucket.get(robot_id)
                if lease is not None and lease.route_id == route_id and resource not in desired_set:
                    del bucket[robot_id]
                    if not bucket:
                        del self.owners[resource]

            self.clear_wait(robot_id)
            return True, set()

    def same_direction_owners(self, resource: str, direction: str, robot_id: str) -> Set[str]:
        if resource not in self.directional_resources or not direction:
            return set()
        with self.lock:
            return {
                rid for rid, lease in self._resource_leases_unlocked(resource).items()
                if rid != robot_id and lease.direction == direction
            }

    def active_directions(self, resource: str, robot_id: str = '') -> Set[str]:
        with self.lock:
            return {
                lease.direction
                for rid, lease in self._resource_leases_unlocked(resource).items()
                if rid != robot_id and lease.direction
            }

    def release_except(
        self,
        robot_id: str,
        route_id: str,
        keep: Iterable[str],
    ):
        keep_set = set(keep)
        with self.lock:
            for resource in list(self.owners):
                bucket = self.owners[resource]
                lease = bucket.get(robot_id)
                if lease is not None and lease.route_id == route_id and resource not in keep_set:
                    del bucket[robot_id]
                    if not bucket:
                        del self.owners[resource]

    def owned_resources(self, robot_id: str, route_id: str) -> List[str]:
        with self.lock:
            return [
                resource for resource, bucket in self.owners.items()
                if (lease := bucket.get(robot_id)) is not None and lease.route_id == route_id
            ]

    def release_route(self, robot_id: str, route_id: str):
        with self.lock:
            for resource in list(self.owners):
                bucket = self.owners[resource]
                lease = bucket.get(robot_id)
                if lease is not None and lease.route_id == route_id:
                    del bucket[robot_id]
                    if not bucket:
                        del self.owners[resource]
            self.clear_wait(robot_id)

    def set_wait(self, robot_id: str, blockers: Set[str]):
        with self.lock:
            if blockers:
                self.wait_for[robot_id] = set(blockers)
                self.wait_since.setdefault(robot_id, time.monotonic())
            else:
                self.clear_wait(robot_id)

    def clear_wait(self, robot_id: str):
        self.wait_for.pop(robot_id, None)
        self.wait_since.pop(robot_id, None)

    def detect_cycle(self) -> List[str]:
        with self.lock:
            graph = {k: set(v) for k, v in self.wait_for.items()}
        visiting: Set[str] = set()
        visited: Set[str] = set()
        stack: List[str] = []

        def dfs(node: str) -> Optional[List[str]]:
            if node in visiting:
                i = stack.index(node)
                return stack[i:] + [node]
            if node in visited:
                return None
            visiting.add(node)
            stack.append(node)
            for nxt in graph.get(node, set()):
                cycle = dfs(nxt)
                if cycle:
                    return cycle
            stack.pop()
            visiting.remove(node)
            visited.add(node)
            return None

        for node in list(graph):
            cycle = dfs(node)
            if cycle:
                return cycle
        return []

    def resolve_cycle_by_yield(self, cycle: Sequence[str], seconds: float = 2.0) -> Optional[str]:
        if not cycle:
            return None
        robots = list(dict.fromkeys(cycle))
        with self.lock:
            victim = max(robots, key=lambda r: self.wait_since.get(r, -math.inf))
            self.yield_until[victim] = time.monotonic() + seconds
            self.clear_wait(victim)
        self.logger.warning(f'deadlock watchdog: {" -> ".join(cycle)}; {victim} yields and replans')
        return victim

    def must_yield(self, robot_id: str) -> float:
        with self.lock:
            return max(0.0, self.yield_until.get(robot_id, 0.0) - time.monotonic())


class TrafficManager(Node):
    def __init__(self):
        super().__init__('traffic_manager')
        p = self.declare_parameter
        self.robot_ids = list(p('robot_ids', ['robot_1', 'robot_2']).value)
        self.wait_retry = float(p('wait_retry', 1.0).value)
        self.max_wait = float(p('max_wait', 90.0).value)
        self.nav_timeout = float(p('nav_timeout', 300.0).value)
        self.max_replans = int(p('max_replans', 2).value)
        self.lookahead_resources = max(1, int(p('reservation_lookahead_resources', 2).value))

        default_graph = os.path.join(
            get_package_share_directory('warehouse_gazebo'),
            'config',
            'traffic.yaml',
        )
        graph_path = str(p('traffic_file', default_graph).value)
        self.graph = TrafficGraph(graph_path)
        self.directional_min_spacing = float(
            p('directional_min_spacing', self.graph.directional_min_spacing).value
        )
        self.reservations = ReservationTable(
            self.graph.resources,
            self.graph.directional_resources,
            self.graph.resource_capacity,
            self.get_logger(),
        )
        self.states: Dict[str, RobotState] = {}
        self.state_lock = threading.RLock()
        self.active_lock = threading.RLock()
        self.active_robots: Set[str] = set()

        cb = ReentrantCallbackGroup()
        self.nav_clients = {
            rid: ActionClient(
                self,
                NavigateThroughPoses,
                f'/{rid}/navigate_through_poses',
                callback_group=cb,
            )
            for rid in self.robot_ids
        }
        for rid in self.robot_ids:
            self.create_subscription(
                RobotState,
                f'/{rid}/robot_state',
                lambda msg, r=rid: self._on_state(r, msg),
                10,
                callback_group=cb,
            )
        self.server = ActionServer(
            self,
            TrafficNavigate,
            '/fleet/traffic_navigate',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cb,
        )
        self.get_logger().info(
            f'traffic manager up: {len(self.graph.nodes)} graph nodes, '
            f'{sum(len(v) for v in self.graph.adj.values()) // 2} bidirectional edges, '
            f'rolling look-ahead={self.lookahead_resources}, '
            f'directional aisles={sorted(self.graph.directional_resources)}, '
            f'min follower spacing={self.directional_min_spacing:.1f} m, robots {self.robot_ids}'
        )

    def _on_state(self, rid: str, msg: RobotState):
        with self.state_lock:
            self.states[rid] = msg

    def _on_goal(self, request: TrafficNavigate.Goal):
        if request.robot_id not in self.nav_clients:
            self.get_logger().warning(f'traffic request rejected: unknown robot {request.robot_id}')
            return GoalResponse.REJECT
        with self.active_lock:
            if request.robot_id in self.active_robots:
                self.get_logger().warning(
                    f'traffic request rejected: {request.robot_id} already has an active leg'
                )
                return GoalResponse.REJECT
            self.active_robots.add(request.robot_id)
        return GoalResponse.ACCEPT

    @staticmethod
    def _on_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _state_xy(self, rid: str) -> Optional[Point]:
        with self.state_lock:
            s = self.states.get(rid)
            return None if s is None else (float(s.x), float(s.y))

    def _feedback(self, gh, state: str, active_resource: str = '', waiting_for: str = ''):
        fb = TrafficNavigate.Feedback()
        fb.traffic_state = state
        fb.active_resource = active_resource
        fb.waiting_for = waiting_for
        gh.publish_feedback(fb)

    def _pose_msgs(self, points: Sequence[Tuple[float, float, float]]) -> List[PoseStamped]:
        out = []
        stamp = self.get_clock().now().to_msg()
        for x, y, yaw in points:
            p = PoseStamped()
            p.header.frame_id = self.graph.frame_id
            p.header.stamp = stamp
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            out.append(p)
        return out

    @staticmethod
    def _fill_nav_goal(goal: NavigateThroughPoses.Goal, poses: List[PoseStamped]):
        """Jazzy uses PoseStamped[]; newer Nav2 uses nav_msgs/Goals."""
        if hasattr(goal.poses, 'goals'):
            goal.poses.header.frame_id = 'map'
            goal.poses.goals = poses
        else:
            goal.poses = poses

    def _navigate_through(
        self,
        rid: str,
        poses: List[PoseStamped],
        traffic_gh,
    ) -> Tuple[bool, str]:
        client = self.nav_clients[rid]
        if not client.wait_for_server(timeout_sec=5.0):
            return False, 'NavigateThroughPoses server unavailable'

        goal = NavigateThroughPoses.Goal()
        self._fill_nav_goal(goal, poses)
        send = client.send_goal_async(goal)
        t0 = time.monotonic()

        while not send.done():
            if traffic_gh.is_cancel_requested:
                return False, 'canceled'
            if time.monotonic() - t0 > 10.0:
                return False, 'NavigateThroughPoses goal acknowledgement timeout'
            time.sleep(0.05)

        handle = send.result()
        if handle is None or not handle.accepted:
            return False, 'NavigateThroughPoses goal rejected'

        result = handle.get_result_async()
        while not result.done():
            if traffic_gh.is_cancel_requested:
                handle.cancel_goal_async()
                return False, 'canceled'
            if time.monotonic() - t0 > self.nav_timeout:
                handle.cancel_goal_async()
                return False, 'NavigateThroughPoses timeout'
            time.sleep(0.1)

        wrapped = result.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            return True, 'done'
        msg = getattr(wrapped.result, 'error_msg', '') or f'Nav2 status {wrapped.status}'
        return False, msg

    def _chunk_target(
        self,
        plan: RoutePlan,
        final_goal: Tuple[float, float, float],
        edge_count: int,
    ) -> Tuple[float, float, float]:
        """Target at the end of the next resource chunk.

        A direction-owned rack aisle may span multiple collinear graph edges; those edges are driven
        continuously as one chunk so the robot does not stop at the artificial midpoint node.
        """
        edge_count = max(1, min(edge_count, len(plan.edges)))
        edge = plan.edges[edge_count - 1]
        x, y = self.graph.nodes[edge.dst]
        if edge_count >= len(plan.edges):
            return final_goal

        nx, ny = self.graph.nodes[plan.edges[edge_count].dst]
        yaw = math.atan2(ny - y, nx - x)
        return float(x), float(y), float(yaw)

    def _follower_spacing_blockers(
        self,
        rid: str,
        request: ResourceRequest,
    ) -> Set[str]:
        """Same-direction followers must have physical separation before aisle admission."""
        owners = self.reservations.same_direction_owners(
            request.resource,
            request.direction,
            rid,
        )
        if not owners:
            return set()

        mine = self._state_xy(rid)
        if mine is None:
            return owners

        blockers: Set[str] = set()
        with self.state_lock:
            for owner in owners:
                state = self.states.get(owner)
                if state is None:
                    blockers.add(owner)
                    continue
                distance = math.hypot(float(state.x) - mine[0], float(state.y) - mine[1])
                if distance < self.directional_min_spacing:
                    blockers.add(owner)
        return blockers

    def _reserve_plan_window(
        self,
        rid: str,
        route_id: str,
        plan: RoutePlan,
    ) -> Tuple[bool, List[ResourceRequest], Set[str], bool]:
        """Reserve one plan's rolling window.

        Returns (ok, requests, blockers, spacing_wait). spacing_wait is True when a same-direction
        follower should simply wait for separation instead of triggering an alternate-route search.
        """
        if not plan.edges:
            return True, [], set(), False

        first_request = self.graph.edge_request(plan.edges[0])
        if first_request.resource in self.graph.directional_resources:
            same_direction = self.reservations.same_direction_owners(
                first_request.resource,
                first_request.direction,
                rid,
            )
            if same_direction:
                spacing_blockers = self._follower_spacing_blockers(rid, first_request)
                if spacing_blockers:
                    return False, [], spacing_blockers, True

                follower_window = self.graph.follower_requests(plan)
                ok, blockers = self.reservations.try_set_window(
                    follower_window,
                    rid,
                    route_id,
                )
                return ok, follower_window, blockers, False

        full_window = self.graph.window_requests(
            plan,
            start_edge=0,
            lookahead_resources=self.lookahead_resources,
        )
        ok, blockers = self.reservations.try_set_window(full_window, rid, route_id)
        return ok, full_window, blockers, False

    def _try_reserve_next_window(
        self,
        rid: str,
        route_id: str,
        start: Point,
        goal: Tuple[float, float, float],
    ) -> Tuple[Optional[RoutePlan], List[ResourceRequest], Set[str]]:
        """Try the stable shortest route first, then reroute only for a real resource conflict."""
        primary = self.graph.plan(start, (goal[0], goal[1]), set())
        if primary is None:
            return None, [], set()
        if not primary.edges:
            return primary, [], set()

        ok, requests, blockers, spacing_wait = self._reserve_plan_window(
            rid,
            route_id,
            primary,
        )
        if ok:
            return primary, requests, set()

        # Same-direction bunching is a transient condition. Do not send a follower around the
        # warehouse merely because it is temporarily too close to the robot ahead.
        if spacing_wait:
            return None, [], blockers

        all_blockers = set(blockers)
        blocked_now = self.reservations.blocked_resources(rid)
        alternate = self.graph.plan(start, (goal[0], goal[1]), blocked_now)

        if alternate is not None and alternate.edges:
            primary_edges = [e.edge_id for e in primary.edges]
            alternate_edges = [e.edge_id for e in alternate.edges]
            if alternate_edges != primary_edges:
                alt_ok, alt_requests, alt_blockers, alt_spacing_wait = self._reserve_plan_window(
                    rid,
                    route_id,
                    alternate,
                )
                if alt_ok:
                    self.get_logger().info(
                        f'{rid}: rolling reservation reroute: '
                        f'{primary_edges} -> {alternate_edges}'
                    )
                    return alternate, alt_requests, set()
                all_blockers |= alt_blockers
                if alt_spacing_wait:
                    return None, [], all_blockers

        return None, [], all_blockers

    def _execute(self, gh):
        req = gh.request
        rid = req.robot_id
        route_id = f'{rid}:{req.task_id}:{req.stage}:{time.monotonic_ns()}'

        result = TrafficNavigate.Result()
        result.success = False
        result.route_edges = []

        goal = (float(req.goal_x), float(req.goal_y), float(req.goal_yaw))
        traversed_edges: List[str] = []
        last_traversed_resource = ''
        wait_started: Optional[float] = None

        try:
            while rclpy.ok():
                if gh.is_cancel_requested:
                    result.message = 'canceled'
                    gh.canceled()
                    return result

                delay = self.reservations.must_yield(rid)
                if delay > 0.0:
                    self._feedback(gh, 'DEADLOCK_YIELD')
                    time.sleep(min(delay, self.wait_retry))
                    continue

                start = self._state_xy(rid)
                if start is None:
                    if wait_started is None:
                        wait_started = time.monotonic()
                    if time.monotonic() - wait_started >= self.max_wait:
                        result.message = 'no robot state received'
                        gh.abort()
                        return result
                    self._feedback(gh, 'WAITING_FOR_ROBOT_STATE')
                    time.sleep(self.wait_retry)
                    continue

                plan, window, blockers = self._try_reserve_next_window(
                    rid,
                    route_id,
                    start,
                    goal,
                )

                # We are already on the final graph node. This can happen for a pure orientation
                # adjustment or because the actual pose is within anchor_snap_dist of the endpoint.
                if plan is not None and not plan.edges:
                    self.reservations.release_route(rid, route_id)
                    poses = self._pose_msgs([goal])
                    nav_ok, nav_msg = self._navigate_through(rid, poses, gh)
                    if nav_ok:
                        result.success = True
                        result.message = 'done'
                        result.route_edges = traversed_edges
                        gh.succeed()
                        return result
                    if gh.is_cancel_requested or nav_msg == 'canceled':
                        result.message = 'canceled'
                        gh.canceled()
                        return result
                    result.message = nav_msg
                    gh.abort()
                    return result

                if plan is None:
                    # The robot is stationary at a graph node. Drop speculative future reservations
                    # so they do not create hold-and-wait cycles, but conservatively retain:
                    #   * the node currently occupied, if it was previously reserved;
                    #   * the resource just traversed, because the robot may still overlap it.
                    unconstrained = self.graph.plan(start, (goal[0], goal[1]), set())
                    keep: Set[str] = set()
                    if unconstrained is not None and unconstrained.nodes:
                        keep.add(f'node:{unconstrained.nodes[0]}')
                    if last_traversed_resource:
                        keep.add(last_traversed_resource)
                    self.reservations.release_except(rid, route_id, keep)

                    self.reservations.set_wait(rid, blockers)
                    cycle = self.reservations.detect_cycle()
                    if cycle:
                        self.reservations.resolve_cycle_by_yield(cycle)

                    if wait_started is None:
                        wait_started = time.monotonic()
                    if time.monotonic() - wait_started >= self.max_wait:
                        result.message = (
                            'traffic wait timeout: '
                            + (','.join(sorted(blockers)) if blockers else 'reserved corridor')
                        )
                        gh.abort()
                        return result

                    waiting = ','.join(sorted(blockers)) or 'reserved corridor'
                    active = ','.join(self.reservations.owned_resources(rid, route_id))
                    self._feedback(
                        gh,
                        'WAITING_FOR_TRAFFIC',
                        active_resource=active,
                        waiting_for=waiting,
                    )
                    time.sleep(self.wait_retry)
                    continue

                wait_started = None
                edge_count = self.graph.chunk_edge_count(plan)
                chunk_edges = plan.edges[:edge_count]
                target = self._chunk_target(plan, goal, edge_count)
                result.route_edges = traversed_edges + [e.edge_id for e in plan.edges]
                window_names = self.graph.format_requests(window)

                self.get_logger().info(
                    f'{rid}: rolling route {req.stage}; '
                    f'next={[e.edge_id for e in chunk_edges]}, '
                    f'window={window_names}, '
                    f'remaining={[e.edge_id for e in plan.edges]}'
                )
                self._feedback(
                    gh,
                    'NAVIGATING',
                    active_resource=','.join(window_names),
                )

                # Direction-owned aisles are driven as one continuous physical chunk. Ordinary
                # resources still execute one graph edge at a time.
                nav_ok = False
                nav_msg = ''
                for attempt in range(self.max_replans + 1):
                    current = self._state_xy(rid) or start
                    points = self.graph.dense_segment_points(current, target)
                    poses = self._pose_msgs(points)
                    nav_ok, nav_msg = self._navigate_through(rid, poses, gh)

                    if gh.is_cancel_requested or nav_msg == 'canceled':
                        result.message = 'canceled'
                        gh.canceled()
                        return result
                    if nav_ok:
                        break

                    self.get_logger().warning(
                        f'{rid}: traffic-approved chunk {[e.edge_id for e in chunk_edges]} failed: {nav_msg} '
                        f'(attempt {attempt + 1}/{self.max_replans + 1})'
                    )
                    if attempt < self.max_replans:
                        # Keep the full current rolling window while recovering: a failed Nav2
                        # action may leave the robot part-way through the edge.
                        self._feedback(
                            gh,
                            'REPLANNING',
                            active_resource=','.join(
                                self.reservations.owned_resources(rid, route_id)
                            ),
                        )
                        time.sleep(self.wait_retry)

                if not nav_ok:
                    result.message = nav_msg
                    gh.abort()
                    return result

                traversed_edges.extend(e.edge_id for e in chunk_edges)
                result.route_edges = list(traversed_edges)
                last_traversed_resource = chunk_edges[-1].resource

                # The current window is deliberately kept here. On the next iteration
                # try_set_window() acquires the new look-ahead first and only then releases the
                # resource behind the robot.
                time.sleep(0.05)

        finally:
            self.reservations.release_route(rid, route_id)
            with self.active_lock:
                self.active_robots.discard(rid)


def main():
    rclpy.init()
    node = TrafficManager()
    executor = MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Send a NavigateToPose goal to one robot's Nav2 stack.

Stations come from warehouse_gazebo/config/stations.yaml (docks, pick_N, drop_N), poses in 'map'.
"""
import argparse
import math
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def load_stations():
    """
    load warehouse stations.yaml
    """
    path = get_package_share_directory('warehouse_gazebo') + '/config/stations.yaml'
    with open(path) as f:
        data = yaml.safe_load(f)
    return {s['id']: s for key in ('docks', 'pick_stations', 'drop_stations') for s in data[key]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('robot', help='namespace, e.g. robot_1')
    ap.add_argument('target', nargs='?', help='station id, e.g. pick_1 / drop_2 / dock_1')
    ap.add_argument('--xy', nargs='+', type=float, metavar='V', help='X Y [YAW] in the map frame')
    args = ap.parse_args()

    if args.xy:
        if len(args.xy) not in (2, 3):
            ap.error('--xy takes X Y [YAW]')
        x, y, yaw = args.xy[0], args.xy[1], (args.xy[2] if len(args.xy) == 3 else 0.0)
    elif args.target:
        stations = load_stations()
        if args.target not in stations:
            sys.exit(f'unknown station {args.target!r}; known: {", ".join(sorted(stations))}')
        s = stations[args.target]
        x, y, yaw = s['x'], s['y'], s['yaw']
    else:
        ap.error('give a station id or --xy X Y [YAW]')

    rclpy.init()
    node = Node('send_goal')
    client = ActionClient(node, NavigateToPose, f'/{args.robot}/navigate_to_pose')
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error(f'/{args.robot}/navigate_to_pose not available (Nav2 up and active?)')
        sys.exit(1)

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'          # stamp 0 = "latest transform"
    goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

    last = [0.0]

    def feedback(msg):
        """
        """
        now = node.get_clock().now().nanoseconds * 1e-9
        if now - last[0] > 1.0:
            last[0] = now
            node.get_logger().info(f'distance remaining: {msg.feedback.distance_remaining:.2f} m')

    node.get_logger().info(f'{args.robot} -> ({x:.2f}, {y:.2f}, yaw {yaw:.2f})')
    future = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        node.get_logger().error('goal rejected')
        sys.exit(1)
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    status = result_future.result().status      # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
    node.get_logger().info({4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, f'status {status}'))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if status == 4 else 2)


if __name__ == '__main__':
    main()

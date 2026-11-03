#!/usr/bin/env python3
"""Submit random pick -> drop tasks to the fleet manager (load generator for tests and scale runs).

    ros2 run warehouse_fleet_manager submit_random_tasks --count 10 --interval 5 --seed 1
    ros2 run warehouse_fleet_manager submit_random_tasks --pickup pick_1 --drop drop_4 --count 1
"""
import argparse
import random
import sys

import rclpy
from rclpy.node import Node
from warehouse_msgs.srv import SubmitTask


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--count', type=int, default=5)
    ap.add_argument('--interval', type=float, default=5.0, help='seconds between submissions')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--pickup', help='fixed pickup station (default: random pick_1..pick_4)')
    ap.add_argument('--drop', help='fixed drop station (default: random drop_1..drop_4)')
    args = ap.parse_args()
    rng = random.Random(args.seed)

    rclpy.init()
    node = Node('submit_random_tasks')
    client = node.create_client(SubmitTask, '/fleet/submit_task')
    if not client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error('/fleet/submit_task not available - is the fleet manager running?')
        sys.exit(1)
    for i in range(args.count):
        req = SubmitTask.Request()
        req.pickup_id = args.pickup or f'pick_{rng.randint(1, 4)}'
        req.drop_id = args.drop or f'drop_{rng.randint(1, 4)}'
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        res = fut.result()
        node.get_logger().info(f'{req.pickup_id} -> {req.drop_id}: '
                               f'{"accepted as " + res.task_id if res and res.accepted else "REJECTED"}')
        if i < args.count - 1:
            end = node.get_clock().now().nanoseconds * 1e-9 + args.interval
            while node.get_clock().now().nanoseconds * 1e-9 < end:
                rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

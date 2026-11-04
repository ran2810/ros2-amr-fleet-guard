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


PICKUPS = [f'pick_{i}' for i in range(1, 5)]
DROPS = [f'drop_{i}' for i in range(1, 5)]


def build_balanced_tasks(count, rng, fixed_pickup=None, fixed_drop=None):
    """Return up to ``count`` evenly covered pickup/drop pairs.

    A balanced cycle contains every allowed pickup/drop combination exactly once.
    The order inside each cycle is shuffled using ``rng`` so a fixed --seed stays
    deterministic while station coverage remains balanced.

    If --pickup or --drop is fixed, balancing is performed over the remaining
    free dimension. If both are fixed, the same pair is repeated.
    """
    pickups = [fixed_pickup] if fixed_pickup else PICKUPS
    drops = [fixed_drop] if fixed_drop else DROPS
    combinations = [(pickup, drop) for pickup in pickups for drop in drops]

    tasks = []
    while len(tasks) < count:
        cycle = combinations.copy()
        rng.shuffle(cycle)
        tasks.extend(cycle[:count - len(tasks)])
    return tasks


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--count', type=int, default=5)
    ap.add_argument('--interval', type=float, default=5.0,
                    help='seconds between submissions')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--pickup',
                    help='fixed pickup station (default: random pick_1..pick_4)')
    ap.add_argument('--drop',
                    help='fixed drop station (default: random drop_1..drop_4)')
    ap.add_argument(
        '--balanced',
        action='store_true',
        help=(
            'generate balanced pickup/drop coverage instead of independent random choices.\n'
            'With no fixed stations, each block of 16 tasks contains every\n'
            'pick_1..pick_4 x drop_1..drop_4 combination exactly once.\n'
            'The order is shuffled; --seed makes it reproducible.'
        ),
    )
    args = ap.parse_args()

    if args.count < 1:
        ap.error('--count must be >= 1')
    if args.interval < 0.0:
        ap.error('--interval must be >= 0')
    if args.pickup and args.pickup not in PICKUPS:
        ap.error(f'--pickup must be one of: {", ".join(PICKUPS)}')
    if args.drop and args.drop not in DROPS:
        ap.error(f'--drop must be one of: {", ".join(DROPS)}')

    rng = random.Random(args.seed)

    if args.balanced:
        tasks = build_balanced_tasks(
            args.count,
            rng,
            fixed_pickup=args.pickup,
            fixed_drop=args.drop,
        )
    else:
        tasks = [
            (
                args.pickup or f'pick_{rng.randint(1, 4)}',
                args.drop or f'drop_{rng.randint(1, 4)}',
            )
            for _ in range(args.count)
        ]

    rclpy.init()
    node = Node('submit_random_tasks')
    client = node.create_client(SubmitTask, '/fleet/submit_task')

    if not client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error(
            '/fleet/submit_task not available - is the fleet manager running?')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    mode = 'balanced' if args.balanced else 'random'
    node.get_logger().info(
        f'generating {len(tasks)} tasks in {mode} mode'
        + (f' (seed={args.seed})' if args.seed is not None else '')
    )

    for i, (pickup_id, drop_id) in enumerate(tasks):
        req = SubmitTask.Request()
        req.pickup_id = pickup_id
        req.drop_id = drop_id

        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        res = fut.result()

        node.get_logger().info(
            f'{req.pickup_id} -> {req.drop_id}: '
            f'{"accepted as " + res.task_id if res and res.accepted else "REJECTED"}'
        )

        if i < len(tasks) - 1:
            end = node.get_clock().now().nanoseconds * 1e-9 + args.interval
            while node.get_clock().now().nanoseconds * 1e-9 < end:
                rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

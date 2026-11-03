#!/usr/bin/env python3
"""Live table of the fleet: state, battery, nearest neighbour, task counters.

    ros2 run warehouse_fleet_manager fleet_monitor
"""
import math

import rclpy
from rclpy.node import Node
from warehouse_fleet_manager.manager import PALETTE
from warehouse_msgs.msg import FleetStatus

NAMES = {0: 'IDLE', 1: 'TO_PICK', 2: 'PICK', 3: 'TO_DROP', 4: 'DROP', 5: 'TO_DOCK', 6: 'CHARGE', 7: 'FAULT', 8: 'STARTING'}
HEALTH = {0: 'ok', 1: 'degraded', 2: 'FAULTED'}


class Monitor(Node):
    def __init__(self):
        super().__init__('fleet_monitor')
        self.create_subscription(FleetStatus, '/fleet/status', self._show, 10)

    def _show(self, msg):
        lines = ['\033[2J\033[H' + f'queued {msg.tasks_queued}  active {msg.tasks_active}  '
                 f'done {msg.tasks_done}  failed {msg.tasks_failed}', '',
                 f'{"  robot":<9}{"state":<10}{"health":<10}{"battery":>8}{"nearest":>9}  {"task":<22}detail']
        for r in sorted(msg.robots, key=lambda r: r.robot_id):
            near = '-' if math.isinf(r.nearest_robot_m) else f'{r.nearest_robot_m:.1f} m'
            red, green, blue = (int(255 * c) for c in PALETTE[(int(r.robot_id.split('_')[-1]) - 1) % len(PALETTE)])
            swatch = f'\033[38;2;{red};{green};{blue}m\u25a0\033[0m {r.robot_id:<7}'   # colour block = the robot's colour
            lines.append(f'{swatch}{NAMES.get(r.state, r.state):<10}{HEALTH.get(r.health, r.health):<10}'
                         f'{r.battery:7.0f}%{near:>9}  {r.assigned_task:<22}{r.detail}')
        print('\n'.join(lines), flush=True)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

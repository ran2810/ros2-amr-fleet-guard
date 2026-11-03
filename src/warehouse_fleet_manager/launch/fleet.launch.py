"""Robot agents (one per robot) + the fleet manager.

    ros2 launch warehouse_fleet_manager fleet.launch.py num_robots:=4

Usually started through fleet_sim.launch.xml together with the simulator and Nav2.
Battery numbers are accelerated on purpose; see warehouse_robot_agent/agent.py for what each does.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BATTERY_ARGS = {
    'initial_soc': ('100.0', 'Starting state of charge [%] (lower it to see charging sooner)'),
    'drain_per_meter': ('0.25', 'Battery drain per metre driven [%/m]'),
    'drain_idle': ('0.005', 'Battery drain while idle [%/s]'),
    'charge_rate': ('0.5', 'Charging rate on the dock [%/s]'),
    'min_mission_soc': ('35.0', 'Agent refuses missions below this SOC [%]'),
    'min_battery': ('40.0', 'Fleet manager only assigns to robots at or above this SOC [%]'),
}


def _setup(context):
    n = int(LaunchConfiguration('num_robots').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    ids = [f'robot_{i + 1}' for i in range(n)]
    value = lambda name: float(LaunchConfiguration(name).perform(context))  # noqa: E731
    nodes = []
    for i, rid in enumerate(ids):
        nodes.append(Node(
            package='warehouse_robot_agent', executable='robot_agent', name='robot_agent',
            namespace=rid, output='screen',
            parameters=[{'robot_id': rid, 'dock_id': f'dock_{i + 1}', 'robot_ids': ids,
                         'use_sim_time': use_sim_time,
                         'initial_soc': value('initial_soc'),
                         'drain_per_meter': value('drain_per_meter'),
                         'drain_idle': value('drain_idle'),
                         'charge_rate': value('charge_rate'),
                         'min_mission_soc': value('min_mission_soc')}]))
    nodes.append(Node(
        package='warehouse_fleet_manager', executable='fleet_manager', name='fleet_manager',
        output='screen',
        parameters=[{'robot_ids': ids, 'use_sim_time': use_sim_time,
                     'min_battery': value('min_battery')}]))
    return nodes


def generate_launch_description():
    args = [DeclareLaunchArgument('num_robots', default_value='2'),
            DeclareLaunchArgument('use_sim_time', default_value='true')]
    args += [DeclareLaunchArgument(k, default_value=v[0], description=v[1]) for k, v in BATTERY_ARGS.items()]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])

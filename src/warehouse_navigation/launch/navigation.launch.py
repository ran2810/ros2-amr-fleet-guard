"""Start one independent Nav2 stack (map_server, AMCL, planner, controller, BT, ...) per robot.

    ros2 launch warehouse_navigation navigation.launch.py num_nav_robots:=1

Why this does not reuse nav2_bringup's launch files? --> those remap /tf -> tf inside the robot
namespace (one TF tree per robot). This project uses ONE global /tf with prefixed frames
(robot_1/odom, robot_1/base_link, ...), so the Nav2 nodes are started here without any tf
remapping, and AMCL's map -> robot_N/odom lands on the same /tf as everything else.

Per robot N:   /robot_N/{amcl,map_server,planner_server,controller_server,behavior_server,
               bt_navigator,velocity_smoother}  +  two lifecycle managers
Command path:  controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel -> Gazebo bridge
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAX_ROBOTS = 8


def _render_params(template_path, ns, dock, use_sim_time):
    """Fill the per-robot placeholders, force use_sim_time everywhere, wrap under the namespace."""
    with open(template_path) as f:
        text = f.read()
    text = (text.replace('__NS__', ns)
                .replace('__X__', str(dock['x']))
                .replace('__Y__', str(dock['y']))
                .replace('__YAW__', str(dock['yaw'])))
    data = yaml.safe_load(text)

    def inject(node):
        for key, value in node.items():
            if key == 'ros__parameters':
                value['use_sim_time'] = use_sim_time
            elif isinstance(value, dict):
                inject(value)
    inject(data)

    path = os.path.join(tempfile.gettempdir(), f'nav2_{ns}_{os.getpid()}.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump({ns: data}, f, sort_keys=False)   # top-level key = namespace
    return path


def _nav_nodes(ns, params, map_yaml, use_sim_time, lifecycle_delay):
    def nav2(package, executable, name=None, remappings=None, extra=None):
        return Node(package=package, executable=executable, name=name or executable,
                    namespace=ns, output='screen',
                    parameters=[params, {'use_sim_time': use_sim_time}, *(extra or [])],
                    remappings=remappings or [])

    def lifecycle(name, nodes):
        return Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                    name=name, namespace=ns, output='screen',
                    parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                                 'bond_timeout': 8.0, 'node_names': nodes}])

    return [
        nav2('nav2_map_server', 'map_server', extra=[{'yaml_filename': map_yaml}]),
        nav2('nav2_amcl', 'amcl'),
        nav2('nav2_controller', 'controller_server', remappings=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_planner', 'planner_server'),
        nav2('nav2_behaviors', 'behavior_server', remappings=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_bt_navigator', 'bt_navigator'),
        nav2('nav2_velocity_smoother', 'velocity_smoother',
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        # Lifecycle managers start late and one after the other: bringing every node up at the same
        # moment overloaded a CPU-only machine ("async_send_request failed", bringup aborted).
        TimerAction(period=lifecycle_delay, actions=[
            lifecycle('lifecycle_manager_localization', ['map_server', 'amcl'])]),
        TimerAction(period=lifecycle_delay + 4.0, actions=[
            lifecycle('lifecycle_manager_navigation',
                      ['controller_server', 'planner_server', 'behavior_server',
                       'bt_navigator', 'velocity_smoother'])]),
    ]


def _launch_setup(context):
    n = int(LaunchConfiguration('num_nav_robots').perform(context))
    if not 0 <= n <= MAX_ROBOTS:
        raise RuntimeError(f'num_nav_robots must be 0..{MAX_ROBOTS}, got {n}')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    wait_timeout = LaunchConfiguration('odom_wait_timeout').perform(context)
    lifecycle_delay = float(LaunchConfiguration('lifecycle_delay').perform(context))

    nav_pkg = get_package_share_directory('warehouse_navigation')
    gz_pkg = get_package_share_directory('warehouse_gazebo')
    template = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(nav_pkg, 'maps', 'warehouse.yaml')
    with open(os.path.join(gz_pkg, 'config', 'stations.yaml')) as f:
        docks = yaml.safe_load(f)['docks']

    actions = []
    for i in range(n):
        ns = f'robot_{i + 1}'
        params = _render_params(template, ns, docks[i], use_sim_time)
        # Nav2 needs odom->base_link + scans to exist; wait for the first odom message
        # (i.e. the robot has been spawned) instead of guessing a delay.
        wait_odom = ExecuteProcess(
            cmd=['timeout', wait_timeout, 'ros2', 'topic', 'echo', '--once',
                 '--field', 'header.frame_id', f'/{ns}/odom'],
            name=f'wait_{ns}_odom', output='log')
        actions.append(wait_odom)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=wait_odom,
            on_exit=_nav_nodes(ns, params, map_yaml, use_sim_time, lifecycle_delay))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_nav_robots', default_value='1',
                              description=f'Robots 1..N get their own Nav2 stack (0..{MAX_ROBOTS})'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('lifecycle_delay', default_value='4.0',
                              description='Seconds after the Nav2 nodes start before the lifecycle managers run '
                                          '(navigation manager runs 4 s later); raise on a loaded machine'),
        DeclareLaunchArgument('odom_wait_timeout', default_value='240',
                              description='Max seconds to wait for a robot to appear before starting its Nav2'),
        OpaqueFunction(function=_launch_setup),
    ])

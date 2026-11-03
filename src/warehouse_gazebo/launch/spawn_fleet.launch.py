"""Spawn N namespaced AMRs into a running Gazebo 'warehouse' world.

Per robot i (namespace robot_i):
    * robot_state_publisher   /robot_i/robot_description, TF frames prefixed "robot_i/"
    * ros_gz_sim create       spawns the model on dock i (pose from config/stations.yaml)
    * ros_gz_bridge           /robot_i/{scan,odom,imu,joint_states,cmd_vel} and /robot_i/tf -> /tf
    * static TF map -> robot_i/odom  

TF strategy: ONE global /tf, one shared 'map' frame, every other frame is
prefixed with the robot namespace. Nothing is published unqualified.
"""
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAX_ROBOTS = 8

# (urdf rgba, rviz 0-255 rgb) - one distinguishable colour per robot
PALETTE = [
    ((0.15, 0.45, 0.85, 1.0), (38, 115, 217)),
    ((0.85, 0.20, 0.20, 1.0), (217, 51, 51)),
    ((0.20, 0.70, 0.30, 1.0), (51, 179, 77)),
    ((0.90, 0.75, 0.10, 1.0), (230, 191, 26)),
    ((0.60, 0.25, 0.80, 1.0), (153, 64, 204)),
    ((0.10, 0.75, 0.80, 1.0), (26, 191, 204)),
    ((0.95, 0.45, 0.10, 1.0), (242, 115, 26)),
    ((0.90, 0.40, 0.65, 1.0), (230, 102, 166)),
]


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('1', 'true', 'yes', 'on')


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _rviz_config(num_robots, num_nav_robots=0):
    """Write an RViz config that matches the number of spawned robots."""
    topic = lambda value, reliability, durability: {  # noqa: E731
        'Value': value, 'Depth': 5, 'History Policy': 'Keep Last',
        'Reliability Policy': reliability, 'Durability Policy': durability}
    displays = [
        {'Class': 'rviz_default_plugins/Grid', 'Name': 'Grid', 'Enabled': True,
         'Cell Size': 1, 'Plane Cell Count': 40},
        {'Class': 'rviz_default_plugins/TF', 'Name': 'TF', 'Enabled': True,
         'Show Names': False, 'Show Axes': True, 'Show Arrows': False, 'Marker Scale': 0.4},
    ]
    for i in range(num_robots):
        ns = f'robot_{i + 1}'
        r, g, b = PALETTE[i][1]
        displays.append({
            'Class': 'rviz_default_plugins/RobotModel', 'Name': f'{ns} model', 'Enabled': True,
            'Description Topic': topic(f'/{ns}/robot_description', 'Reliable', 'Transient Local'),
            # rsp prefixes TF frames, not the URDF link names -> tell RViz the prefix
            'TF Prefix': ns, 'Alpha': 1, 'Visual Enabled': True, 'Collision Enabled': False})
        displays.append({
            'Class': 'rviz_default_plugins/LaserScan', 'Name': f'{ns} scan', 'Enabled': True,
            'Topic': topic(f'/{ns}/scan', 'Best Effort', 'Volatile'),
            'Style': 'Flat Squares', 'Size (m)': 0.04, 'Color Transformer': 'FlatColor',
            'Color': f'{r}; {g}; {b}', 'Use Fixed Frame': True, 'Decay Time': 0})
    for i in range(num_nav_robots):
        ns = f'robot_{i + 1}'
        displays.insert(0, {
            'Class': 'rviz_default_plugins/Map', 'Name': f'{ns} map', 'Enabled': True,
            'Topic': topic(f'/{ns}/map', 'Reliable', 'Transient Local'),
            'Alpha': 0.7, 'Color Scheme': 'map'})
        for label, name in (('global costmap', 'global_costmap'), ('local costmap', 'local_costmap')):
            displays.append({
                'Class': 'rviz_default_plugins/Map', 'Name': f'{ns} {label}', 'Enabled': label == 'global costmap',
                'Topic': topic(f'/{ns}/{name}/costmap', 'Reliable', 'Transient Local'),
                'Alpha': 0.4, 'Color Scheme': 'costmap'})
        displays.append({
            'Class': 'rviz_default_plugins/Path', 'Name': f'{ns} plan', 'Enabled': True,
            'Topic': topic(f'/{ns}/plan', 'Reliable', 'Volatile'),
            'Color': '25; 255; 0', 'Line Style': 'Lines'})
        displays.append({   # AMCL particles (nav2_rviz_plugins, part of ros-jazzy-navigation2); best-effort QoS
            'Class': 'nav2_rviz_plugins/ParticleCloud', 'Name': f'{ns} particles', 'Enabled': True,
            'Topic': topic(f'/{ns}/particle_cloud', 'Best Effort', 'Volatile')})
    displays.append({   # fleet manager: stations, task routes per robot, queue (empty without the manager)
        'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'fleet tasks', 'Enabled': True,
        'Topic': topic('/fleet/markers', 'Reliable', 'Volatile')})
    tools = [
        {'Class': 'rviz_default_plugins/MoveCamera'},
        {'Class': 'rviz_default_plugins/Select'},
        {'Class': 'rviz_default_plugins/FocusCamera'},
        {'Class': 'rviz_default_plugins/Measure'},
    ]
    if num_nav_robots:   # the toolbar tools publish for robot_1 (fleet manager will do the rest later)
        tools += [
            {'Class': 'rviz_default_plugins/SetInitialPose',
             'Topic': topic('/robot_1/initialpose', 'Reliable', 'Volatile')},
            {'Class': 'rviz_default_plugins/SetGoal',
             'Topic': topic('/robot_1/goal_pose', 'Reliable', 'Volatile')},
        ]
    config = {
        'Panels': [
            {'Class': 'rviz_common/Displays', 'Name': 'Displays'},
            {'Class': 'rviz_common/Views', 'Name': 'Views'},
        ],
        'Visualization Manager': {
        'Class': '', 'Name': 'root', 'Enabled': True, 'Displays': displays,
        # without a camera tool the mouse cannot pan/zoom/rotate the view
        'Tools': tools,
        'Global Options': {'Fixed Frame': 'map', 'Frame Rate': 20, 'Background Color': '40; 44; 52'},
        'Views': {'Current': {
            'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': 'Current View',
            'Scale': 28, 'X': 0, 'Y': 0, 'Angle': 0}}}}
    path = os.path.join(tempfile.gettempdir(), f'warehouse_fleet_{os.getpid()}.rviz')
    with open(path, 'w') as f:
        yaml.safe_dump(config, f, sort_keys=False)
    return path


def _launch_setup(context):
    num_robots = int(_arg(context, 'num_robots'))
    if not 1 <= num_robots <= MAX_ROBOTS:
        raise RuntimeError(f'num_robots must be 1..{MAX_ROBOTS}, got {num_robots}')

    use_sim_time = _flag(context, 'use_sim_time')
    static_map_tf = _flag(context, 'publish_static_map_tf')
    num_nav = int(_arg(context, 'num_nav_robots'))
    gz_pkg = get_package_share_directory('warehouse_gazebo')
    desc_pkg = get_package_share_directory('warehouse_description')
    xacro_path = os.path.join(desc_pkg, 'urdf', 'amr.urdf.xacro')

    with open(os.path.join(gz_pkg, 'config', 'stations.yaml')) as f:
        stations = yaml.safe_load(f)
    world_name = stations['world_name']
    docks = stations['docks']

    always = []   # start immediately (no dependency on Gazebo being up)
    spawns = []   # start once the Gazebo world accepts create requests

    for i in range(num_robots):
        ns = f'robot_{i + 1}'
        dock = docks[i]
        rgba, _ = PALETTE[i]

        description = xacro.process_file(xacro_path, mappings={
            'robot_ns': ns,
            'accent_rgba': ' '.join(str(c) for c in rgba),
            'lidar_rate': _arg(context, 'lidar_rate'),
            'lidar_samples': _arg(context, 'lidar_samples'),
            'imu_rate': _arg(context, 'imu_rate'),
        }).toxml()

        always.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': description,
                         'frame_prefix': f'{ns}/',
                         'use_sim_time': use_sim_time}]))

        always.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'{ns}_bridge', output='screen',
            arguments=[
                f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                f'/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            # every robot's odom->base_link lands on the single global /tf
            remappings=[(f'/{ns}/tf', '/tf')],
            parameters=[{'use_sim_time': use_sim_time}]))

        if static_map_tf and i >= num_nav:
            # DiffDrive odometry starts at 0 at spawn, so odom frame == spawn pose in map.
            # Robots with a Nav2 stack skip this: AMCL publishes their map -> odom.
            always.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name=f'{ns}_map_to_odom', output='log',
                arguments=['--x', str(dock['x']), '--y', str(dock['y']), '--z', '0',
                           '--roll', '0', '--pitch', '0', '--yaw', str(dock['yaw']),
                           '--frame-id', 'map', '--child-frame-id', f'{ns}/odom']))

        spawn = Node(
            package='ros_gz_sim', executable='create', name=f'spawn_{ns}', output='screen',
            arguments=['-world', world_name, '-name', ns,
                       '-topic', f'/{ns}/robot_description',
                       '-x', str(dock['x']), '-y', str(dock['y']), '-z', '0.1',
                       '-Y', str(dock['yaw'])])
        # stagger spawns: gentler on a CPU-only machine
        spawns.append(TimerAction(period=i * float(_arg(context, 'spawn_stagger')),
                                  actions=[spawn]))

    # Wait until the world's create service is advertised (instead of guessing a delay).
    wait_for_gz = ExecuteProcess(
        cmd=['timeout', _arg(context, 'gz_wait_timeout'), 'bash', '-c',
             f'until gz service -l 2>/dev/null | grep -q "/world/{world_name}/create$"; do sleep 1; done'],
        name='wait_for_gazebo', output='screen')
    on_gz_ready = RegisterEventHandler(OnProcessExit(target_action=wait_for_gz, on_exit=spawns))

    actions = [*always, wait_for_gz, on_gz_ready]

    if _flag(context, 'rviz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', _rviz_config(num_robots, num_nav)],
            parameters=[{'use_sim_time': use_sim_time}]))
    return actions


def generate_launch_description():
    declare = lambda name, default, doc: DeclareLaunchArgument(name, default_value=default,  # noqa: E731
                                                              description=doc)
    return LaunchDescription([
        declare('num_robots', '2', f'Number of AMRs to spawn (1..{MAX_ROBOTS})'),
        declare('use_sim_time', 'true', 'Use /clock from Gazebo'),
        declare('rviz', 'true', 'Start RViz with a config generated for num_robots'),
        declare('publish_static_map_tf', 'true',
                'Publish static map->robotN/odom for robots that have no Nav2/AMCL'),
        declare('num_nav_robots', '0',
                'Robots 1..N run Nav2 (AMCL owns their map->odom, RViz shows nav displays)'),
        declare('lidar_rate', '10', 'LiDAR update rate [Hz]'),
        declare('lidar_samples', '360', 'LiDAR beams per scan'),
        declare('imu_rate', '50', 'IMU update rate [Hz]'),
        declare('spawn_stagger', '1.5', 'Seconds between consecutive robot spawns'),
        declare('gz_wait_timeout', '120', 'Max seconds to wait for Gazebo before attempting spawn'),
        OpaqueFunction(function=_launch_setup),
    ])

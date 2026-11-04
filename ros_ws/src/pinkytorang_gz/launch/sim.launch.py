"""핑키토랑 Gazebo(Harmonic) 시뮬

  gz 서버 1개 + 로봇 N대. 로봇마다 ROS_DOMAIN_ID를 따로 둬서 실제 구성(서버 50, pinky1=51 ...)과 같게 만든다.
  각 로봇 도메인 안에는 실제 로봇처럼 /cmd_vel /odom /tf /scan /imu /joint_states /camera/image_raw 가 있다
  (네임스페이스 없음). 그래서 Nav2 · fleet_agent · domain_bridge(system.launch.xml)를 그대로 쓴다.

  ros2 launch pinkytorang_gz sim.launch.py                         # 3대 (config/robots.yaml), GUI
  ros2 launch pinkytorang_gz sim.launch.py only:=pinky1            # 한 대만
  ros2 launch pinkytorang_gz sim.launch.py nav:=true               # + 로봇별 Nav2(map v5, nav_v5_sim) + 초기 위치
  ros2 launch pinkytorang_gz sim.launch.py gui:=false              # 서버만 (헤드리스)
  ros2 launch pinkytorang_gz sim.launch.py agent:=true             # + 로봇별 fleet_agent (관제 서버와 연결)
"""
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, AppendEnvironmentVariable, TimerAction)
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'pinkytorang_gz'


def _truthy(v):
    return str(v).lower() in ('1', 'true', 'yes', 'on')


def _bridge_yaml(rid):
    """gz /<rid>/... <-> ROS /... (로봇 도메인 안)"""
    g = f'/{rid}'
    entries = [
        ('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS'),
        (f'{g}/tf', '/tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V', 'GZ_TO_ROS'),
        (f'{g}/odom', '/odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
        (f'{g}/cmd_vel', '/cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ'),
        (f'{g}/joint_states', '/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model', 'GZ_TO_ROS'),
        (f'{g}/scan', '/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan', 'GZ_TO_ROS'),
        (f'{g}/imu', '/imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS'),
        (f'{g}/camera/image_raw', '/camera/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image', 'GZ_TO_ROS'),
        (f'{g}/camera/camera_info', '/camera/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo',
         'GZ_TO_ROS'),
    ]
    data = [{'gz_topic_name': gz, 'ros_topic_name': ros, 'ros_type_name': rt, 'gz_type_name': gt, 'direction': d}
            for gz, ros, rt, gt, d in entries]
    path = os.path.join(tempfile.gettempdir(), f'pinkytorang_gz_{rid}_bridge.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def _robot_description(rid, ctx):
    import xacro
    share = get_package_share_directory(PKG)
    doc = xacro.process_file(os.path.join(share, 'urdf', 'pinky_tray.urdf.xacro'), mappings={
        'gz_ns': f'{rid}/',
        'with_tray': LaunchConfiguration('with_tray').perform(ctx),
        'cam_tilt_deg': LaunchConfiguration('cam_tilt_deg').perform(ctx),
        'lamp_plugin': LaunchConfiguration('lamp_plugin').perform(ctx),
        'camera': LaunchConfiguration('camera').perform(ctx),
    })
    return doc.toxml()


def _robots(ctx):
    share = get_package_share_directory(PKG)
    with open(LaunchConfiguration('robots_file').perform(ctx)) as f:
        robots = yaml.safe_load(f)['robots']
    with open(os.path.join(share, 'config', 'spawn_poses.yaml')) as f:
        spawn = yaml.safe_load(f)
    only = [s.strip() for s in LaunchConfiguration('only').perform(ctx).split(',') if s.strip()]
    out = []
    for r in robots:
        if only and r['id'] not in only:
            continue
        if 'x' in r:
            pose = (float(r['x']), float(r['y']), float(r.get('yaw', 0.0)))
        else:
            p = spawn[r['start_node']]
            pose = (p['x'], p['y'], p['yaw'])
        out.append((r['id'], int(r['domain']), pose, r.get('start_node', '')))
    return out


def _spawn_all(ctx):
    share = get_package_share_directory(PKG)
    nav = _truthy(LaunchConfiguration('nav').perform(ctx))
    agent = _truthy(LaunchConfiguration('agent').perform(ctx))
    if agent and not nav:
        nav = True   # fleet_agent는 Nav2가 있어야 움직임
    fake_guide = LaunchConfiguration('fake_guide').perform(ctx)
    map_yaml = LaunchConfiguration('map').perform(ctx)
    nav_params = LaunchConfiguration('nav_params').perform(ctx)
    actions = []
    for i, (rid, dom, (x, y, yaw), start_node) in enumerate(_robots(ctx)):
        desc = _robot_description(rid, ctx)

        def in_domain(acts, delay=None, dom=dom):
            # ROS_DOMAIN_ID를 이 로봇 것으로 바꾼 묶음. TimerAction 안에서 묶어야 늦게 뜨는 것도 같은 도메인으로 뜬다
            g = GroupAction([SetEnvironmentVariable('ROS_DOMAIN_ID', str(dom)), *acts], scoped=True, forwarding=True)
            return TimerAction(period=delay, actions=[g]) if delay else g

        actions.append(in_domain([
            Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': desc, 'use_sim_time': True}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
                 parameters=[{'config_file': _bridge_yaml(rid), 'use_sim_time': True}]),
        ]))
        # 로봇마다 1.5초 간격으로 스폰 (gz가 뜰 시간 + 동시 스폰 충돌 방지)
        actions.append(in_domain([
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', rid, '-topic', 'robot_description',
                            '-x', f'{x:.4f}', '-y', f'{y:.4f}', '-z', '0.01', '-Y', f'{yaw:.4f}'])],
            delay=3.0 + 1.5 * i))
        if nav:
            actions.append(in_domain([
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('pinky_navigation'), 'launch', 'gz_bringup_launch.xml')),
                    # use_composition False: 컴포저블 로딩은 launch 프로세스의 ROS 노드(도메인 하나)로 서비스를
                    # 불러서, 로봇마다 도메인이 다르면 다른 로봇 컨테이너에 잘못 실린다 -> 노드를 프로세스로 따로 띄움
                    launch_arguments={'map': map_yaml, 'params_file': nav_params,
                                      'use_sim_time': 'True', 'use_composition': 'False'}.items())],
                delay=6.0 + 1.5 * i))
            # AMCL 초기 위치 = 스폰 자리 (AMCL이 구독할 때까지 기다렸다 1초 간격 3번)
            qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
            msg = ('{header: {frame_id: map}, pose: {pose: {position: {x: %.4f, y: %.4f}, '
                   'orientation: {z: %.6f, w: %.6f}}, covariance: [0.0025,0,0,0,0,0, 0,0.0025,0,0,0,0, '
                   '0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.01]}}' % (x, y, qz, qw))
            actions.append(in_domain([
                ExecuteProcess(cmd=['ros2', 'topic', 'pub', '-w', '1', '-t', '3', '-r', '1', '/initialpose',
                                    'geometry_msgs/msg/PoseWithCovarianceStamped', msg], output='screen')],
                delay=10.0 + 1.5 * i))
        if agent:
            # fleet_agent (+ obstacle). 카메라 · 정밀 정차는 시뮬에 마커 · 바닥선이 없어서 끔
            actions.append(in_domain([
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('fleet_agent'), 'launch', 'fleet_agent.launch.xml')),
                    launch_arguments={'robot_id': rid, 'start_node': start_node,
                                      'params': os.path.join(share, 'params', 'fleet_agent_sim.yaml'),
                                      'use_camera': 'false', 'use_docking': 'false', 'use_guide': 'false',
                                      'fake_guide': fake_guide}.items())],
                delay=14.0 + 1.5 * i))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PKG)
    desc_parent = os.path.dirname(get_package_share_directory('pinky_description'))
    world_default = os.path.join(share, 'worlds', 'pinkytorang_v5.sdf')

    def gz(ctx):
        world = LaunchConfiguration('world').perform(ctx)
        gui = _truthy(LaunchConfiguration('gui').perform(ctx))
        args = f'-r -v 3 {world}' if gui else f'-r -s -v 3 {world}'
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items())]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_default),
        DeclareLaunchArgument('robots_file', default_value=os.path.join(share, 'config', 'robots.yaml')),
        DeclareLaunchArgument('only', default_value='', description='쉼표로 로봇 id (빈칸 = 전부)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('nav', default_value='false', description='로봇별 Nav2 + AMCL 초기 위치'),
        DeclareLaunchArgument('agent', default_value='false',
                              description='로봇별 fleet_agent (+ obstacle). 켜면 nav도 켜짐. fleet_agent 패키지 필요'),
        DeclareLaunchArgument('fake_guide', default_value='false', description='안내 손님 확인을 가짜로 (카메라 없음)'),
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'config', 'pinkytorang_map_v5.yaml')),
        DeclareLaunchArgument('nav_params', default_value=os.path.join(share, 'params', 'nav_v5_sim.yaml')),
        DeclareLaunchArgument('with_tray', default_value='true'),
        DeclareLaunchArgument('cam_tilt_deg', default_value='8'),
        DeclareLaunchArgument('lamp_plugin', default_value='false'),
        DeclareLaunchArgument('camera', default_value='true', description='false = 카메라 센서 끔 (가벼움)'),

        # model://pinky_table, package://pinky_description · pinkytorang_gz 메시 찾는 경로
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', desc_parent),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(share)),

        OpaqueFunction(function=gz),
        OpaqueFunction(function=_spawn_all),
    ])

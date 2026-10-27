"""서버 노트북에서 로봇별 domain_bridge를 함께 띄운다.

실행:
  source ~/ros2_ws/install/setup.bash          # fleet_msgs 포함
  ros2 launch <이 폴더>/fleet_bridge.launch.py                    # 기본: pinky1, pinky2, pinky3
  ros2 launch <이 폴더>/fleet_bridge.launch.py robots:=pinky1,pinky2         # 일부만

robots에 적은 각 로봇마다 같은 폴더의 <robot_id>_bridge.yaml이 있어야 한다.
(없으면 make_bridge_config.py로 먼저 만든다)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def make_bridges(context):
    robots = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',') if r.strip()]
    nodes = []
    for robot in robots:
        cfg = os.path.join(HERE, f'{robot}_bridge.yaml')
        if not os.path.exists(cfg):
            raise RuntimeError(f'{cfg} 가 없습니다. make_bridge_config.py {robot} <도메인> 으로 먼저 만드세요.')
        nodes.append(Node(
            package='domain_bridge',
            executable='domain_bridge',
            name=f'{robot}_fleet_bridge',
            arguments=[cfg],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='pinky1,pinky2,pinky3',
                              description='브리지할 로봇 ID 목록 (쉼표로 구분)'),
        OpaqueFunction(function=make_bridges),
    ])

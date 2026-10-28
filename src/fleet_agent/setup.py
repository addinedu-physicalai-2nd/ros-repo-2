from glob import glob

from setuptools import setup

package_name = 'fleet_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.docking'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='todo@example.com',
    description='로봇 쪽 fleet 에이전트',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_agent = fleet_agent.agent_node:main',
            'docking_node = fleet_agent.docking_node:main',
            'obstacle_node = fleet_agent.obstacle_node:main',
            'initial_pose = fleet_agent.initial_pose:main',     # 시작 도킹 좌표로 AMCL 초기 위치
            'marker_dock = fleet_agent.docking.marker_dock:main',   # 혼자 실행: --calibrate / --record / --dry-run
        ],
    },
)

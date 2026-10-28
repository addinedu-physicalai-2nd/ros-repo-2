"""시작 노드(도킹) 좌표로 AMCL 초기 위치를 한 번 넣는다 (RViz 2D Pose Estimate 대신).

node_graph.yaml에서 start_node의 x, y, yaw를 읽어 /initialpose로 보낸다.
AMCL이 /initialpose를 구독할 때까지 기다렸다가 몇 번 보내고 끝난다.
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPose(Node):
    def __init__(self):
        super().__init__('initial_pose')
        self.declare_parameter('start_node', '')
        self.declare_parameter('node_graph', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('repeat', 3)        # 보내는 횟수 (1초 간격)
        self.declare_parameter('timeout_sec', 60.0)  # AMCL을 이만큼 기다림

        self.start_node = self.get_parameter('start_node').value
        self.map_frame = self.get_parameter('map_frame').value
        self.left = int(self.get_parameter('repeat').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.done = False
        self.pose = self.find_pose(self.get_parameter('node_graph').value, self.start_node)
        if self.pose is None:
            self.done = True
            return

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(f'{self.start_node} {self.pose} 로 초기 위치를 넣을게요 (AMCL 기다리는 중)')

    def find_pose(self, path, node_id):
        if not node_id or not path:
            self.get_logger().warn('start_node 또는 node_graph가 비어 있어 초기 위치를 안 넣음 (RViz로 맞춰 주세요)')
            return None
        try:
            with open(path, encoding='utf-8') as f:
                nodes = yaml.safe_load(f)['nodes']
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'node_graph 읽기 실패: {path} ({e})')
            return None
        for n in nodes:
            if n['id'] == node_id:
                return float(n['x']), float(n['y']), float(n.get('yaw', 0.0))
        self.get_logger().error(f'node_graph에 {node_id} 없음')
        return None

    def tick(self):
        waited = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        if self.pub.get_subscription_count() == 0:   # AMCL이 아직 안 뜸
            if waited > self.timeout:
                self.get_logger().error('AMCL이 안 떠서 초기 위치를 못 넣음 (RViz로 맞춰 주세요)')
                self.done = True
            return
        x, y, yaw = self.pose
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        m.pose.covariance[0] = 0.05 ** 2    # x
        m.pose.covariance[7] = 0.05 ** 2    # y
        m.pose.covariance[35] = 0.1 ** 2    # yaw
        self.pub.publish(m)
        self.left -= 1
        if self.left <= 0:
            self.get_logger().info(f'초기 위치 넣음: {self.start_node} ({x:.3f}, {y:.3f}, yaw {yaw:.3f})')
            self.done = True


def main():
    rclpy.init()
    node = InitialPose()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

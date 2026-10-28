"""시작 노드(도킹) 좌표로 AMCL 초기 위치를 한 번 넣는다 (RViz 2D Pose Estimate 대신).

node_graph.yaml에서 start_node의 x, y, yaw를 읽어 /initialpose로 보낸다.
AMCL이 /initialpose를 구독할 때까지 기다렸다가 몇 번 보낸다.
spin_deg > 0 이면 그다음 제자리에서 그만큼 돌아 AMCL이 위치를 다시 맞추게 한다 (시험용, 도킹 마커 없이 시작할 때)
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class InitialPose(Node):
    def __init__(self):
        super().__init__('initial_pose')
        self.declare_parameter('start_node', '')
        self.declare_parameter('node_graph', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('repeat', 3)        # 보내는 횟수 (1초 간격)
        self.declare_parameter('timeout_sec', 60.0)  # AMCL을 이만큼 기다림
        any_type = ParameterDescriptor(dynamic_typing=True)   # launch에서 "360"은 정수, "-1.57"은 실수로 들어와서
        self.declare_parameter('start_yaw', '', any_type)     # 비우면 node_graph의 yaw (없으면 0). 숫자(rad)면 그 방향으로
        self.declare_parameter('spin_deg', 0.0, any_type)     # 초기 위치 뒤 제자리 회전 (0 = 안 돎, 360 = 한 바퀴)
        self.declare_parameter('spin_speed', 0.6, any_type)   # rad/s
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.start_node = self.get_parameter('start_node').value
        self.map_frame = self.get_parameter('map_frame').value
        self.left = int(self.get_parameter('repeat').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.done = False
        self.spin_left = math.radians(float(self.get_parameter('spin_deg').value))
        self.spin_speed = abs(float(self.get_parameter('spin_speed').value)) or 0.6
        self.pose = self.find_pose(self.get_parameter('node_graph').value, self.start_node)
        if self.pose is None:
            self.done = True
            return
        yaw_txt = str(self.get_parameter('start_yaw').value).strip()
        if yaw_txt:
            self.pose = (self.pose[0], self.pose[1], float(yaw_txt))

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.spinning = False
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
        if self.spinning:
            return
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
            if self.spin_left > 0:
                self.start_spin()
            else:
                self.done = True

    # ---- 제자리 회전: 0.1초마다 각속도를 보내고, 돈 각도(속도 x 시간)가 spin_deg가 되면 멈춘다
    def start_spin(self):
        self.spinning = True
        self.spin_last = self.get_clock().now()
        self.get_logger().info(f'위치 맞추려고 제자리 {math.degrees(self.spin_left):.0f}도 돌게요')
        self.spin_timer = self.create_timer(0.1, self.spin_tick)

    def spin_tick(self):
        now = self.get_clock().now()
        dt = (now - self.spin_last).nanoseconds / 1e9
        self.spin_last = now
        self.spin_left -= self.spin_speed * dt
        t = Twist()
        if self.spin_left > 0:
            t.angular.z = self.spin_speed
            self.cmd.publish(t)
            return
        for _ in range(3):
            self.cmd.publish(t)   # 0 = 멈춤
        self.spin_timer.cancel()
        self.get_logger().info('회전 끝')
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

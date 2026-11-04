"""Obstacle 노드: 진행 방향 LiDAR로 막힘을 보고, 사람(DYNAMIC) / 물건(STATIC)을 가른다

받음 /scan (sensor_msgs/LaserScan)
     /obstacle/direction (std_msgs/String)  fleet_agent가 지금 가는 방향: FORWARD / REVERSE / STOP
     /obstacle/person (std_msgs/Bool)       guide 노드가 막혔을 때 카메라 YOLO로 본 결과 (앞에 toy_person이 있나)
보냄 /obstacle/status (std_msgs/String, JSON, scan마다)
     {"blocked": bool, "kind": "PERSON" | "DYNAMIC" | "STATIC" | "UNKNOWN" | "", "distance": m, "direction": ...}

- FORWARD면 앞쪽, REVERSE면 뒤쪽 부채꼴(sector_deg)에서 가장 가까운 거리를 본다. STOP이면 막힘 없음으로 보고
- 앞 · 뒤 멈춤 거리는 따로 (v7 바구니가 뒤라 뒤쪽이 더 길다)
- 종류 판단은 core.ObstacleClass (멈춘 뒤 거리가 흔들리면 사람, 2초 그대로면 물건)
- 제자리 회전 중(/cmd_vel 직진 0, 회전만)에는 막힘 없음: 앞으로 안 가니 앞 물체에 부딪힐 일 없음
  (실측: N7에서 T4 모서리 쪽으로 도는 중 테이블 모서리가 15 cm로 부채꼴에 들어와 멈춤 -> BLOCKED STATIC)
  멈춰 있을 때(직진 0 · 회전 0)는 그대로 봄 (막혀서 선 상태가 풀렸다 걸렸다 하지 않게)
- 카메라(YOLO) 사람 판별: 앞으로 갈 때 막히면 guide 노드가 1초에 한 번 YOLO -> /obstacle/person.
  person_hold_sec 안에 true가 왔으면 PERSON (guide 노드 · 모델이 없으면 예전처럼 LiDAR로만)
- 카메라 막힘 /obstacle/camera (std_msgs/Bool): 작은 레고 사람은 LiDAR 높이 아래라 안 잡혀 밀고 감 ->
  앞으로 갈 때 guide 노드가 YOLO로 '바로 앞 손님'(박스가 크고 가운데)을 보면 true -> LiDAR가 못 봐도 막힘(PERSON).
  false가 오면 바로 풀림 (사람 치우면 출발). 결과가 camera_hold_sec 넘게 안 오면 저절로 풀림
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .core import CameraBlock, ObstacleClass, wrap


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle')
        p = self.declare_parameter
        scan = p('scan_topic', '/scan').value
        self.scan_yaw = p('scan_yaw_offset', 0.0).value          # LiDAR 0도가 로봇 앞에서 돌아간 각 (rad)
        self.sector = math.radians(p('sector_deg', 30.0).value)  # 진행 방향 +- 이 각도 안만 본다
        self.stop_front = p('stop_distance_forward', 0.18).value  # 로봇 중심 기준 (앞 몸체 0.06 + 여유)
        self.stop_rear = p('stop_distance_reverse', 0.28).value   # 뒤 바구니 0.15 + 여유
        self.range_min = p('range_min_valid', 0.03).value
        self.cls = ObstacleClass(window=p('dynamic_window_sec', 1.5).value,
                                 move_thresh=p('dynamic_move_m', 0.04).value,
                                 static_sec=p('static_sec', 2.0).value)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/obstacle/status', 10)
        self.create_subscription(String, '/obstacle/direction', self.on_direction, latched)
        self.create_subscription(LaserScan, scan, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, p('cmd_topic', '/cmd_vel').value, self.on_cmd, 10)
        self.spin_v = p('spin_ignore_v', 0.01).value    # 이보다 느린 직진 + spin_ignore_w보다 빠른 회전 = 제자리 회전
        self.spin_w = p('spin_ignore_w', 0.05).value
        self.cmd = (0.0, 0.0, -1.0)    # v, w, 받은 시각
        self.spinning = False
        # 회전이 막 끝났을 때 (노드에서 돌고 출발): 벽 모서리가 잠깐 멈춤 거리 안에 들어와 멈춤 · 출발을 반복하던 것
        #   -> 회전 끝 뒤 after_spin_sec 동안은 LiDAR 막힘이 after_spin_confirm초 이어져야 막힘
        self.after_spin = p('after_spin_sec', 1.0).value
        self.after_spin_confirm = p('after_spin_confirm_sec', 0.3).value
        self.spin_end = -1e9
        self.lidar_since = None
        self.direction = 'STOP'
        self.person_hold = p('person_hold_sec', 2.0).value   # 카메라가 사람을 본 결과를 이만큼 믿음
        self.person_t = -1e9
        self.create_subscription(Bool, '/obstacle/person', self.on_person, 10)
        self.camera = CameraBlock(hold=p('camera_hold_sec', 1.0).value)
        self.camera_on = p('camera_block', True).value
        self.cam_was = False
        self.create_subscription(Bool, '/obstacle/camera', self.on_camera, 10)
        self.get_logger().info('obstacle 노드 시작')

    def on_camera(self, msg):
        if self.camera_on:
            self.camera.update(self.get_clock().now().nanoseconds * 1e-9, msg.data)

    def on_person(self, msg):
        if msg.data:
            self.person_t = self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z, self.get_clock().now().nanoseconds * 1e-9)

    def on_direction(self, msg):
        if msg.data != self.direction:
            self.direction = msg.data
            self.cls.reset()
            self.camera.reset()          # 방향이 바뀌면 예전 카메라 결과는 버림

    def nearest(self, scan, center):
        """center 방향 +- sector 안에서 가장 가까운 거리 (없으면 inf)"""
        best = math.inf
        a = scan.angle_min
        for r in scan.ranges:
            if self.range_min <= r <= scan.range_max and not math.isinf(r) and not math.isnan(r):
                if abs(wrap(a + self.scan_yaw - center)) <= self.sector:
                    best = min(best, r)
            a += scan.angle_increment
        return best

    def on_scan(self, scan):
        now = self.get_clock().now().nanoseconds * 1e-9
        dist = math.inf
        blocked = False
        v, w, t_cmd = self.cmd
        spinning = (self.direction in ('FORWARD', 'REVERSE') and now - t_cmd < 0.5
                    and abs(v) < self.spin_v and abs(w) > self.spin_w)
        if spinning != self.spinning:
            self.spinning = spinning
            self.get_logger().info('제자리 회전 -> 앞 막힘 안 봄' if spinning else '회전 끝 -> 다시 봄')
            self.cls.reset()
            if not spinning:
                self.spin_end = now
        if self.direction in ('FORWARD', 'REVERSE') and not spinning:
            rev = self.direction == 'REVERSE'
            dist = self.nearest(scan, math.pi if rev else 0.0)
            blocked = dist < (self.stop_rear if rev else self.stop_front)
            if blocked:
                self.lidar_since = now if self.lidar_since is None else self.lidar_since
                if now - self.spin_end < self.after_spin and now - self.lidar_since < self.after_spin_confirm:
                    blocked = False      # 회전 막 끝남: 잠깐 스친 것인지 조금 더 봄
            else:
                self.lidar_since = None
        cam = self.direction == 'FORWARD' and not spinning and self.camera.active(now)   # 카메라는 앞만 봄
        if cam != self.cam_was:
            self.cam_was = cam
            self.get_logger().info('카메라: 바로 앞에 사람 -> 멈춤' if cam else '카메라: 앞 사람 없어짐')
        if cam and not blocked:
            blocked = True               # LiDAR엔 안 잡히는 작은 사람
            if math.isinf(dist):
                dist = self.stop_front   # 거리를 모름: 종류 판단용으로 멈춤 거리를 씀
        person = blocked and self.direction == 'FORWARD' and (cam or now - self.person_t < self.person_hold)
        prev = self.cls.person_seen
        kind = self.cls.update(now, dist if blocked else None, person)
        if kind == 'PERSON' and not prev:
            self.get_logger().info(f'장애물 = 사람 (카메라 YOLO, 앞 {dist:.2f} m)')
        out = {'blocked': blocked, 'kind': kind, 'distance': None if math.isinf(dist) else round(dist, 3),
               'direction': self.direction}
        self.status_pub.publish(String(data=json.dumps(out)))


def main():
    rclpy.init()
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

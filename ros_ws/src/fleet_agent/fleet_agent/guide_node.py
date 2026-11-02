"""Guide 노드: 안내 로봇의 손님 확인 (카메라 + YOLO) · 효과. fleet_agent가 요청하고 결과를 받는다.

받음  /guide/request  std_msgs/String JSON {id, action, node}
        action = greet : 앞(카메라)으로 손님을 기다림 -> 찾으면 환영 효과 + ok
                 watch : (fleet_agent가 뒤로 돌려 놓은 상태) 손님이 보이는지 -> ok / 못 찾음
                 arrive: 도착 효과만 (결과 안 보냄)
      /guide/cancel   {id}  기다리던 것 그만
      camera_topic    sensor_msgs/Image (bgr8 / rgb8) 또는 sensor_msgs/CompressedImage
      /obstacle/status  (obstacle 노드) 앞으로 가다 막혔는지
보냄  /guide/result   {id, ok, detail}
      /obstacle/person  std_msgs/Bool: 막혔을 때 obstacle_hz로 YOLO -> 앞에 손님(class_name)이 보이나 (obstacle_check)
      /obstacle/camera  std_msgs/Bool: 앞으로 가는 동안 cam_block_hz로 YOLO -> 바로 앞에 손님이 있나 (cam_block)
                        작은 레고 사람은 LiDAR에 안 잡혀 밀고 가서, 카메라로 직접 막음 (core.near_person)

- YOLO는 요청을 처리하는 동안에만, infer_hz로 가장 최근 프레임 하나씩 돌린다 (CPU 아끼기, 주행 노드와 다른 프로세스)
- confirm 번 연속으로 보이면 "찾음". wait_sec 안에 못 찾으면 ok=false (fleet_agent가 서버에 guest_lost)
- 모델 · ultralytics가 없으면 require_detector=false일 때 바로 ok (detail no_detector) -> 카메라 없이도 안내 흐름 시험 가능
"""
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

from .core import near_box
from .guide_effects import leds_off, play


class GuideNode(Node):
    def __init__(self):
        super().__init__('guide')
        p = self.declare_parameter
        self.camera_topic = p('camera_topic', '/camera/image_raw').value
        self.compressed = p('camera_compressed', False).value
        self.model_path = p('model_path', '').value
        self.class_name = p('class_name', 'PerfectFigure').value
        self.conf = float(p('conf', 0.5).value)
        self.infer_hz = float(p('infer_hz', 4.0).value)
        self.wait_sec = float(p('wait_sec', 30.0).value)
        self.confirm = int(p('confirm', 2).value)
        self.effects = p('effects', True).value
        self.require_detector = p('require_detector', False).value
        self.welcome_first = p('welcome_on_request', True).value   # greet가 오면(테이블 선택) 바로 환영
        self.obstacle_check = p('obstacle_check', True).value       # 앞으로 가다 막히면 YOLO로 사람인지 (obstacle 노드에 알림)
        self.obstacle_hz = float(p('obstacle_hz', 1.0).value)
        self.cam_block = p('cam_block', True).value                  # 앞으로 가는 동안 카메라로 바로 앞 손님을 보면 막음
        self.cam_block_hz = float(p('cam_block_hz', 4.0).value)
        self.cam_block_h = float(p('cam_block_h', 0.20).value)       # 손님 박스 높이가 화면의 이만큼 이상 = 가까움
        self.cam_block_bottom = float(p('cam_block_bottom', 0.85).value)   # 또는 박스 아래끝이 화면의 이만큼 아래 = 바로 앞 바닥
        self.cam_clear_n = int(p('cam_clear_n', 2).value)            # 연속 이만큼 안 보여야 멈춤을 풂 (YOLO가 한 장 놓쳐도 출발 안 하게)
        self.clear_count = 0
        self.cam_block_x = float(p('cam_block_x', 0.30).value)       # 박스 가운데가 화면 가운데에서 이 안 = 가는 길 위
        self.direction = 'STOP'
        self.near = None
        self.last_blocked_check = 0.0
        self.blocked = False
        self.watching = False        # 손님 찾는 중 (그동안 장애물 확인은 쉼)
        self.person_logged = None
        self.model_lock = threading.Lock()                           # 손님 찾기 스레드와 장애물 확인이 모델을 같이 씀

        self.frame = None
        self.frame_t = 0.0
        self.frame_lock = threading.Lock()
        self.job = None              # (id, action, node, cancel_event)
        self.model, self.class_id = self.load_model()

        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.result_pub = self.create_publisher(String, '/guide/result', rel)
        self.create_subscription(String, '/guide/request', self.on_request, rel)
        self.create_subscription(String, '/guide/cancel', self.on_cancel, rel)
        img_type = CompressedImage if self.compressed else Image
        self.create_subscription(img_type, self.camera_topic, self.on_image, qos_profile_sensor_data)   # camera_node = best effort
        self.person_pub = self.create_publisher(Bool, '/obstacle/person', 10)
        self.camera_pub = self.create_publisher(Bool, '/obstacle/camera', 10)
        if (self.obstacle_check or self.cam_block) and self.model is not None:
            self.create_subscription(String, '/obstacle/status', self.on_obstacle, 10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/obstacle/direction', self.on_direction, latched)
            hz = max(self.obstacle_hz, self.cam_block_hz if self.cam_block else 0.0)
            self.create_timer(1.0 / max(0.2, hz), self.check_obstacle)
        self.get_logger().info(f'guide 노드 시작 (카메라 {self.camera_topic}, '
                               f'{"YOLO " + self.class_name if self.model else "감지기 없음"})')

    # ---------------------------------------------------------------- 준비
    def load_model(self):
        if not self.model_path:
            self.get_logger().warn('model_path 없음 -> 손님 확인 없이 바로 OK')
            return None, None
        try:
            from ultralytics import YOLO
            model = YOLO(self.model_path)
            names = model.names
            items = names.items() if isinstance(names, dict) else enumerate(names)
            cid = next(k for k, v in items if v == self.class_name)
            return model, cid
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'YOLO 모델 못 읽음 ({e}) -> 손님 확인 없이 바로 OK')
            return None, None

    # ---------------------------------------------------------------- 장애물이 사람인가 (막혔을 때만)
    def on_obstacle(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        blocked = bool(s.get('blocked')) and s.get('direction') == 'FORWARD'   # 카메라는 앞만 봄
        if blocked and not self.blocked:
            self.person_logged = None
        self.blocked = blocked

    def on_direction(self, msg):
        if msg.data != self.direction:
            self.direction = msg.data
            if self.direction != 'FORWARD' and self.near:
                self.clear_count = self.cam_clear_n
                self.set_near(False)     # 앞으로 안 감: 카메라 막힘 풀기

    def set_near(self, near, box=None):
        """주행 중 YOLO 결과 -> /obstacle/camera. 보이면 바로 멈춤, 풀 때는 cam_clear_n번 연속 안 보여야 (깜빡임에 출발 안 함)"""
        if near:
            self.clear_count = 0
        elif self.near:
            self.clear_count += 1
            if self.clear_count < self.cam_clear_n:
                near = True              # 아직 멈춤 유지
        self.camera_pub.publish(Bool(data=near))
        if near != self.near:
            self.near = near
            if near:
                info = (f' (박스 높이 {box[3] - box[1]:.2f}, 아래끝 {box[3]:.2f}, 가운데 {(box[0] + box[2]) / 2:.2f})'
                        if box else '')
                self.get_logger().info('카메라: 앞에 손님 -> 멈춤' + info)
            elif self.direction == 'FORWARD':
                self.get_logger().info('카메라: 앞에 손님 없음 -> 풂')

    def check_obstacle(self):
        if self.watching:
            return                       # 손님 찾는 중(모델 사용 중)
        fwd = self.cam_block and self.direction == 'FORWARD'
        now = time.monotonic()
        lidar = (self.obstacle_check and self.blocked
                 and now - self.last_blocked_check >= 1.0 / max(0.2, self.obstacle_hz) - 0.01)
        if not (fwd or lidar):
            return                       # 앞으로 안 감 · 안 막힘
        with self.frame_lock:
            msg, ft = self.frame, self.frame_t
        if msg is None or now - ft > 1.0:
            return                       # 최근 프레임 없음 (camera_block은 obstacle 노드에서 저절로 풀림)
        boxes = self.persons(msg)
        if boxes is None:
            return
        if fwd:
            box = near_box(boxes, self.cam_block_h, self.cam_block_x, self.cam_block_bottom)
            self.set_near(box is not None, box)
        if self.blocked and (lidar or fwd):
            self.last_blocked_check = now
            seen = bool(boxes)
            self.person_pub.publish(Bool(data=seen))
            if seen != getattr(self, 'person_logged', None):
                self.person_logged = seen
                self.get_logger().info(f'막힌 앞: {"사람(" + self.class_name + ") 보임" if seen else "사람 안 보임"}')

    def persons(self, msg):
        """프레임 하나에서 class_name 박스들 [(x1, y1, x2, y2) 0~1] (실패하면 None)"""
        try:
            img = self.to_bgr(msg)
            with self.model_lock:
                r = self.model(img, verbose=False, conf=self.conf)[0]
            if r.boxes is None:
                return []
            h, w = img.shape[:2]
            return [(x1 / w, y1 / h, x2 / w, y2 / h)
                    for (x1, y1, x2, y2), c in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist())
                    if int(c) == self.class_id]
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'YOLO 실패: {e}')
            return None

    def detect(self, msg):
        """프레임 하나에 class_name이 보이나 (실패하면 None)"""
        boxes = self.persons(msg)
        return None if boxes is None else bool(boxes)

    def on_image(self, msg):
        with self.frame_lock:
            self.frame = msg
            self.frame_t = time.monotonic()

    def to_bgr(self, msg):
        if self.compressed:
            import cv2
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img

    # ---------------------------------------------------------------- 요청
    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            return
        rid, action, node = req.get('id'), req.get('action'), req.get('node', '')
        if self.job:
            self.job[3].set()            # 새 요청이 오면 이전 것은 그만
        if action == 'arrive':
            self.effect('arrived')      # 효과 끝에 LED 끔 (play finally)
            return
        if action == 'end':             # 안내 끝 (손님 놓침 · 취소 · 복귀): LED 확실히 끔
            if self.effects:
                threading.Thread(target=leds_off, args=(self.get_logger().info,), daemon=True).start()
            return
        if action not in ('greet', 'watch'):
            return
        if action == 'greet' and self.welcome_first:
            self.effect('welcome')               # 테이블을 고르자마자 환영 (손님 찾기는 그다음)
        if self.model is None:
            if self.require_detector:
                self.reply(rid, False, 'no_detector')
            else:
                if action == 'greet' and not self.welcome_first:
                    self.effect('welcome')
                self.reply(rid, True, 'no_detector')
            return
        cancel = threading.Event()
        self.job = (rid, action, node, cancel)
        threading.Thread(target=self.watch, args=(rid, action, node, cancel), daemon=True).start()

    def on_cancel(self, msg):
        try:
            rid = json.loads(msg.data).get('id')
        except ValueError:
            return
        if self.job and self.job[0] == rid:
            self.job[3].set()
            self.get_logger().info(f'안내 확인 취소 (id {rid})')

    # ---------------------------------------------------------------- 손님 찾기 (스레드)
    def watch(self, rid, action, node, cancel):
        self.watching = True
        try:
            self._watch(rid, action, node, cancel)
        finally:
            self.watching = False

    def _watch(self, rid, action, node, cancel):
        self.get_logger().info(f'{node}: 손님 {"기다림" if action == "greet" else "확인"} (최대 {self.wait_sec:.0f}초)')
        end = time.monotonic() + self.wait_sec
        period = 1.0 / max(0.5, self.infer_hz)
        hits, last_t, got_frame = 0, 0.0, False
        while not cancel.is_set() and time.monotonic() < end:
            with self.frame_lock:
                msg, ft = self.frame, self.frame_t
            if msg is None or ft == last_t:
                time.sleep(0.05)
                continue
            last_t, got_frame = ft, True
            t0 = time.monotonic()
            seen = bool(self.detect(msg))
            hits = hits + 1 if seen else 0
            if hits >= self.confirm:
                if action == 'greet' and not self.welcome_first:
                    self.effect('welcome')
                self.reply(rid, True, 'guest_seen')
                return
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
        if cancel.is_set():
            return                       # 취소: 결과 안 보냄 (fleet_agent가 다시 요청함)
        self.reply(rid, False, 'not_seen' if got_frame else 'no_camera')

    def reply(self, rid, ok, detail):
        self.result_pub.publish(String(data=json.dumps({'id': rid, 'ok': ok, 'detail': detail})))
        self.get_logger().info(f'안내 결과 id {rid}: {"OK" if ok else "못 찾음"} ({detail})')

    def effect(self, kind):
        if self.effects:
            threading.Thread(target=play, args=(kind, self.get_logger().info), daemon=True).start()


def main():
    rclpy.init()
    node = GuideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

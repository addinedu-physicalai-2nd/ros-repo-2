"""바닥 테이프 정밀 정차 (P1 픽업대, E자 테이프) + 공용 이동 함수

docking 노드가 부른다
  LineDocker(node, pub, cancelled, tf_buf, ir).run(req)    P1 도착(후진, 축 그대로) -> 바구니를 픽업대에   ('성공'이면 성공)
  LineDocker(...).leave(req)                                 배급 끝 -> 앞으로 빼서 새 선에서 정지 (그다음 Nav2)
  req = precision_stops.yaml 값 + node. 아래 기본값을 req로 덮어쓸 수 있다

E자 테이프 (지도 방향: 북쪽 위, 픽업대 오른쪽)
       새 선   서 선  동 선
        ┃      ┃      ┃         <- 세로선 3개 (북쪽으로)
        ┗━━━━━━┻━━━━━━┛         <- 가로선 (복도를 가로지름)
               ┃                <- 메인선 (복도, 있어도 됨)
  run  : 남쪽을 보고 북쪽으로 후진 -> 가로선(검검검) -> x_after 더 -> 시계 90도 -> 동쪽으로 후진
         (approach=True: Driver가 P1 좌표까지 안 가고 P2에서 바로 맡김 -> 가로선까지 v_approach로)
         -> 동 선(검검검, insert_arm 뒤)에서 정지 = 바구니가 픽업대 위
  leave: 서쪽으로 전진 -> 흰색 본 뒤 검검검을 exit_legs번째(기본 2 = 새 선)에서 정지

바닥 센서: /ir_sensor/range (pinky_sensor_adc main_node), 바닥 230~250 / 테이프 4050~4095
움직임은 odom (TF odom -> base_footprint), 일하는 동안만 /cmd_vel. 주기마다 cancelled()면 Cancelled
"""
import math
import time

from .marker_dock import Cancelled, Docker, clamp, wrap

DEFAULTS = {
    'ir_black': 2000,        # 이보다 크면 검정
    'confirm': 2,            # 새 센서 값이 연속 이만큼 맞아야 인정
    'search_max': 0.30,      # P1 좌표에 선 뒤 가로선 찾을 최대 후진 (축 고정이 아닐 때)
    'approach_max': 1.20,    # 축 고정 후진으로 바로 맡았을 때(P2 -> 가로선) 최대 후진. 못 찾으면 정지
    'v_approach': 0.08,      # 그때 속도 (가로선 감지 뒤 밀린 만큼은 X에서 보정)
    'x_after': 0.035,        # 가로선 감지 후 더 후진 (바구니 높이 맞춤)
    'insert_arm': 0.06,      # 삽입: 이만큼 간 뒤의 검검검만 (서 선 지나가는 것 무시)
    'insert_max': 0.20,
    'exit_legs': 2,          # 빼기: 몇 번째 선에서 멈추나 (1 = 서 선, 2 = 새 선)
    'exit_arm': 0.02,
    'exit_max': 0.35,
    'v_search': 0.05,        # m/s
    'accel': 0.35,           # m/s^2 가감속 제한
}
HZ = 30.0
TIMEOUT_S = 20.0
HEADING_K = 2.0
STOP_LEAD_S = 0.06
LIN_TOL_M = 0.003


class Motion:
    """Docker(odom 이동)에 바닥 센서 + 가감속 직진을 더한 것"""

    def __init__(self, node, pub, cancelled, tf_buf, ir, cfg):
        self.dk = Docker(node, pub, tf_buf, None, moving=True, spin=False, cancelled=cancelled)
        self.ir = ir if ir is not None else {'v': None, 'n': 0}
        self.cfg = cfg
        self.last_v = 0.0

    def send(self, v, w):
        self.dk.send(v, w)
        self.last_v = float(v)

    def stop(self, settle=0.05):
        self.dk.stop(settle)
        self.last_v = 0.0

    def blacks(self):
        v = self.ir.get('v')
        return [int(x > self.cfg['ir_black']) for x in v[:3]] if v else [0, 0, 0]

    def wait_ir(self, timeout=2.0):
        end = time.time() + timeout
        while self.ir.get('v') is None:
            if time.time() > end:
                raise RuntimeError('바닥 센서(/ir_sensor/range)가 안 들어옴. pinky_sensor_adc main_node 켰나요?')
            time.sleep(0.05)

    def drive(self, dist, reverse, heading, vmax, event=None, arm=0.0):
        """heading 유지 직진/후진. event 없으면 dist에서 감속 정지, 있으면 참일 때 정지 (못 찾으면 'max')
        반환 (이유, 간 거리, 이벤트 때 자세)"""
        p0 = self.dk.pose()
        ux, uy = math.cos(heading), math.sin(heading)
        sgn = -1.0 if reverse else 1.0
        acc = self.cfg['accel']
        t0, last_n, hit = time.time(), self.ir['n'], 0
        while time.time() - t0 < TIMEOUT_S:
            p = self.dk.pose()
            s = sgn * ((p[0] - p0[0]) * ux + (p[1] - p0[1]) * uy)
            if event is not None and self.ir['n'] != last_n:
                last_n = self.ir['n']
                hit = hit + 1 if event(self.blacks(), s >= arm) else 0
                if hit >= self.cfg['confirm']:
                    self.stop()
                    return 'event', s, p
            remain = dist - s
            if remain < max(LIN_TOL_M, abs(self.last_v) * STOP_LEAD_S):
                self.stop()
                return ('max' if event else 'done'), s, p
            vm = vmax
            if event is None:
                vm = min(vm, math.sqrt(acc * remain) + 0.02, 2.0 * remain + 0.02)
            dv = acc / HZ
            v = clamp(sgn * vm, self.last_v - dv, self.last_v + dv) if sgn * self.last_v >= 0 else 0.0
            w = clamp(HEADING_K * wrap(heading - p[2]), -0.4, 0.4)
            self.send(v, w)
            time.sleep(1 / HZ)
        self.stop()
        return 'timeout', 0.0, self.dk.pose()

    def move_along(self, dist, heading):
        if abs(dist) > LIN_TOL_M:
            self.drive(abs(dist), dist < 0, heading, self.cfg['v_search'])


class LineDocker:
    def __init__(self, node, pub, cancelled, tf_buf=None, ir=None):
        self.node = node
        self.pub = pub
        self.cancelled = cancelled
        self.tf_buf = tf_buf if tf_buf is not None else getattr(node, 'tf_buf', None)
        self.ir = ir if ir is not None else getattr(node, 'ir', None)

    def _motion(self, req):
        cfg = dict(DEFAULTS)
        cfg.update({k: req[k] for k in DEFAULTS if k in req})
        mv = Motion(self.node, self.pub, self.cancelled, self.tf_buf, self.ir, cfg)
        mv.dk.wait_pose()
        mv.wait_ir()
        return mv, cfg

    def log(self, text):
        self.node.get_logger().info(text)

    def run(self, req):
        """P1: 가로선 찾기 -> X -> 시계 90도 -> 동 선까지 후진 (바구니 삽입)"""
        mv, c = self._motion(req)
        h = mv.dk.pose()[2]                       # 지금 방향 = 복도 축 (남쪽을 봄, 북쪽으로 후진)
        far = bool(req.get('approach'))
        why, s, pe = mv.drive(c['approach_max'] if far else c['search_max'], True, h,
                              c['v_approach'] if far else c['v_search'],
                              event=lambda b, armed: sum(b) == 3)   # 가로선 = 검검검 (메인선은 1~2개)
        if why != 'event':
            return f'실패: 가로선을 {s * 100:.0f} cm 안에서 못 찾음'
        p = mv.dk.pose()
        over = -((p[0] - pe[0]) * math.cos(h) + (p[1] - pe[1]) * math.sin(h))
        mv.move_along(-(c['x_after'] - over), h)
        self.log(f'[tape] 가로선 {s * 100:.1f} cm에서 감지, X {c["x_after"] * 100:.1f} cm (밀림 {over * 1000:+.0f} mm)')
        e = mv.dk.rotate_to(wrap(h - math.pi / 2))
        h_ins = wrap(h - math.pi / 2)
        why, s, _ = mv.drive(c['insert_max'], True, h_ins, c['v_search'],
                             event=lambda b, armed: armed and sum(b) == 3, arm=c['insert_arm'])
        if why != 'event':
            return f'실패: 동 선을 못 찾음 ({s * 100:.1f} cm 후진)'
        self.log(f'[tape] 시계 90° (남은 {e:+.1f}°) -> 삽입 {s * 100:.1f} cm 후진')
        return '성공'

    def leave(self, req):
        """배급 끝: 앞으로 가며 선을 세고 exit_legs번째 선에서 정지 (출발 자리의 동 선은 안 셈)"""
        mv, c = self._motion(req)
        h = mv.dk.pose()[2]
        st = {'white': False, 'legs': 0}

        def ev(b, armed):
            if sum(b) == 0:
                st['white'] = True
            elif sum(b) == 3 and st['white'] and armed:
                st['legs'] += 1
                st['white'] = False
            return st['legs'] >= c['exit_legs']
        why, s, _ = mv.drive(c['exit_max'], False, h, c['v_search'], event=ev, arm=c['exit_arm'])
        if why != 'event':
            return f'실패: {c["exit_legs"]}번째 선을 못 찾음 ({s * 100:.1f} cm 전진)'
        self.log(f'[tape_exit] {s * 100:.1f} cm 전진, {c["exit_legs"]}번째 선에서 정지')
        return '성공'


__all__ = ['LineDocker', 'Motion', 'Cancelled']

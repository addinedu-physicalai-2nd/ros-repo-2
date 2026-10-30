"""주방 통로 정밀 동작: N14 정렬 · 벽 따라 후진 · 도크 들어가기(후면 주차) · 기록

docking 노드가 부른다 (method)
  align (front/axis 있음) : Steps.n14_align     N14에서 마커 4 -> 우90 -> 마커 5 벽면 수직
  corridor               : Steps.follow_bars   축 고정 후진을 라이다 벽 평행으로. 바닥 가로선(N15 · N16)을 밟을 때마다
                                               progress(노드)를 알리고 안 섬. nodes는 달리는 중에 늘어날 수 있음 (APPEND)
                           then = tape   -> P1 가로선에서 line_dock.LineDocker.run (삽입)
                           then = dock_in -> 도크 줄 가로선(N15 / N16)에서 서고 Steps.park_from_line (우90 -> 마커 -> 후면 주차)
  dock_in (corridor 없이) : Steps.dock_in       마커 5 거리로 N15 -> 우90 -> 마커로 후면 주차 (예전 방식)

가로선 (N15 · N16, 검정 테이프 폭 약 2 cm, 센서 값 4095)
  - 바닥 센서 3개 모두 BAR_BLACK(3800) 넘음이 BAR_CONFIRM(2)번 연속이어야 선. 실측: 바닥이 약 20 cm마다 한 번씩
    3개 모두 ~3500으로 튐 (1번) -> 문턱 · 연속 둘 다로 거름
  - 노드 사이 거리 BAR_GAP (바닥 기록 실측). 예상의 EARLY(0.5)도 안 갔는데 나온 선은 무시,
    예상 + MISS_M을 넘어도 안 나오면 거리로 그 노드를 치고 (MISS 뒤 RESYNC_M 안에 나온 선은 그 노드의 늦은 선)
  - P2는 선 없이 N16에서 거리, P1은 E자 가로선

원칙
  - 마커는 멈춰서 재고 움직임은 odom. 가까우면 마커 기울기, 멀면 방위각
  - 통로 후진 방향은 마커 5 벽이 아니라 양옆 벽(라이다)과 평행. 실측: 마커 5 벽이 통로와 약 3.8° 틀어져 있고
    odom도 1 m에 3~4° 휨 -> 벽 기준이 아니면 주방 쪽으로 붙음
  - 후진 중 방위각으로는 방향을 고치지 않음 (옆으로 밀린 만큼 더 밀리는 쪽으로 돌아서)
기록 파일 (~/): align_n14e · align_n14s · align_n15s (마커 5 · 4), dock_target_n15 · dock_target_d3 (마커 3)
  -> ros2 run fleet_agent dock_record all   (docking/dock_record.py)
"""
import json
import math
import os

import numpy as np

from . import marker_dock as md
from . import records
from .undock import _set_size

TURN_DEG = -90.0      # N14 · N15에서 오른쪽으로
V_FAR = 0.08          # 긴 이동
V_NEAR = 0.05         # 자리 맞추기 · 주차
FIX_M = 0.004         # 앞뒤 · 옆 이보다 틀리면 고침
ANG_TOL_DEG = 1.0
TOL_DEG = 0.5         # 정렬(방위각/기울기) 이 안이면 됨
VERIFY_DEG = 2.0
MAX_DEG = 35.0
FRAMES = 8
YAW_OK_Z = 0.22       # 2.5 cm 마커 기준, 이보다 가까우면 기울기(yaw)를 믿음 (크기에 비례)
MATCH_LAT_M = 0.008   # N15 맞추기: 옆으로 이보다 벗어났을 때만 옆걸음 (후진 18 cm로는 1 cm 남짓을 못 고침)
MATCH_DEG = 1.5       # N15 맞추기: 방향 이보다 틀어졌을 때만 회전
FOLLOW_V = 0.08       # 통로 후진 속도 (reverse_speed와 같음)
FOLLOW_STOP_EARLY = 0.12   # P1: 노드 좌표보다 이만큼 앞에서 tape로 넘김 (tape가 가로선 찾기부터)
LIDAR_MAX_M = 1.0     # 라이다 점은 이 거리 안만
LANE_K = 3.0          # 출발선에서 1 cm 벗어나면 1.7° 기울임
LANE_STEER_DEG = 6.0
LANE_W_MAX = 0.25
WALL_ALONG_M = 0.25   # 벽 맞춤: 로봇 앞뒤 이 범위 점만
WALL_MIN_PTS = 15
WALL_MAX_STD = 0.012  # 직선에서 점들이 이보다 흩어지면 그 벽은 안 씀
WALL_GAIN = 0.3       # 스캔 한 번 잰 방향을 얼마나 반영
V_PARK = 0.06         # 후면 주차 최고 속도
PARK_HZ = 15
PARK_ACC = 0.15
PARK_K_LAT = 3.0
PARK_STEER_DEG = 5.0
PARK_K_HEAD = 2.5
PARK_W_MAX = 0.25
PARK_ALPHA = 0.25
# 통로 가로선 (follow_bars)
BAR_BLACK = 3800      # 가로선 문턱 (테이프 4095, 튀는 값 ~3500, 흰 바닥 ~240)
BAR_CONFIRM = 2       # 연속 이만큼 검검검이어야 선
BAR_KIND = {'N15': 'bar', 'N16': 'bar', 'P2': 'dist', 'P1': 'bar'}
BAR_GAP = {'N15': 0.40, 'N16': 0.15, 'P2': 0.48, 'P1': 0.44}   # 앞 노드에서 거리 m (N14->N15 · N15->N16 바닥 기록 실측)
W_REF = 0.173         # 서쪽 칸막이에서 로봇 중심까지 = 통로 가운데 (남쪽을 보면 오른쪽 벽)
EARLY = 0.5
MISS_M = 0.10
RESYNC_M = 0.12
START_GUARD_M = 0.06  # 출발 자리가 선 근처인데 선 위가 아니었으면, 이 안에서 밟는 선은 출발 자리 선
V_BAR_SLOW = 0.04     # 서야 할 선 근처 (그리고 P2 -> P1)
SLOW_M = 0.06


def bearing(m):
    return math.atan2(-m['tx'], md.CAM_X_M + m['tz'])


def _load(path):
    """기록 읽기: 파일 이름만 보고 레포 records/에서 (docking/records.py). '~/'는 예전 표기라 무시"""
    with open(records.find(path)) as f:
        return json.load(f)


def _save(path, d):
    """기록 쓰기: 레포 소스 records/ (커밋 · push하면 모든 로봇이 같이 씀)"""
    p = records.save_path(path)
    with open(p, 'w') as f:
        json.dump(d, f, indent=2)
    md.log(f'  기록 저장 -> {p}')


def bar_update(ir, v):
    """바닥 센서 콜백에서 부름: ir['bar_n'] = 지금까지 밟은 가로선 수, ir['on'] = 지금 선 위"""
    allb = all(x > BAR_BLACK for x in v[:3])
    ir['run'] = ir.get('run', 0) + 1 if allb else 0
    if ir.get('on') is None:
        ir['on'] = allb
        ir.setdefault('bar_n', 0)
    elif allb and not ir['on']:
        if ir['run'] >= BAR_CONFIRM:
            ir['on'] = True
            ir['bar_n'] = ir.get('bar_n', 0) + 1
    elif not allb:
        ir['white'] = ir.get('white', 0) + 1 if ir['on'] else 0
        if ir['on'] and ir['white'] >= 2:
            ir['on'], ir['white'] = False, 0


def align_record(mid, size, m):
    return {'id': mid, 'size': size, 'bearing': bearing(m), 'tz': m['tz'], 'yaw': m['yaw']}


class Lidar:
    """/scan -> 로봇 좌표(base_footprint) 점들. take()는 새 스캔이 왔을 때만 점을 돌려줌"""

    def __init__(self, node, tf_buf):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        self.tf_buf = tf_buf
        self.T = None
        self.last = None
        self.n = 0
        self._taken = 0
        node.create_subscription(LaserScan, '/scan', self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        if self.T is None:
            try:
                from rclpy.time import Time
                t = self.tf_buf.lookup_transform('base_footprint', msg.header.frame_id, Time()).transform
                q = t.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.T = (t.translation.x, t.translation.y, yaw)
            except Exception:
                return
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.03)) & (r < min(msg.range_max, LIDAR_MAX_M))
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        tx, ty, yaw = self.T
        c, s_ = math.cos(yaw), math.sin(yaw)
        pts = np.stack([tx + c * x - s_ * y, ty + s_ * x + c * y], axis=1)
        body = (np.abs(pts[:, 0]) < 0.14) & (np.abs(pts[:, 1]) < 0.09)      # 로봇 자신 (바구니 포함) 빼기
        self.last = pts[~body]
        self.n += 1

    def take(self):
        if self.n == self._taken or self.last is None:
            return None
        self._taken = self.n
        return self.last


def side_lines(pts, along=0.30, near=0.05, far=0.45):
    """로봇 좌표 점 -> 왼쪽 · 오른쪽 벽 직선 맞춤 (x 앞, y 왼쪽). 반환 {side: (거리 m, 각 rad, 점 수)}"""
    out = {}
    for side, sg in (('왼쪽', 1), ('오른쪽', -1)):
        m = (np.abs(pts[:, 0]) < along) & (sg * pts[:, 1] > near) & (sg * pts[:, 1] < far)
        q = pts[m]
        for _ in range(3):
            if len(q) < 8:
                break
            A = np.stack([np.ones(len(q)), q[:, 0]], axis=1)
            (a0, b), *_ = np.linalg.lstsq(A, q[:, 1], rcond=None)
            res = q[:, 1] - (a0 + b * q[:, 0])
            keep = np.abs(res) < max(0.01, 2.5 * np.std(res))
            if keep.all():
                break
            q = q[keep]
        if len(q) >= 8:
            out[side] = (abs(a0) / math.hypot(1, b), math.atan(b), len(q), float(np.std(res)))
    return out


class Steps:
    """마커 · 라이다 · 바닥 센서로 하는 정밀 동작 모음. 움직임은 odom (Motion), 마커는 멈춰서 잼"""

    def __init__(self, mv, cam, log, lidar=None):
        self.mv, self.cam, self.log = mv, cam, log
        self.lidar = lidar
        mv.dk.cam = cam

    def see(self, mid, size, frames=FRAMES, find=False):
        _set_size(size)
        self.cam.id = mid
        self.mv.stop(0.15)
        m = self.cam.measure_stable(frames=frames)
        if m is None and find and self.mv.dk.find_marker():
            self.mv.stop(0.15)
            m = self.cam.measure_stable(frames=frames)
        return m

    def _marker_odom(self, m):
        p = self.mv.dk.pose()
        return p, np.array(p[:2]) + md.rot(p[2]) @ np.array([md.CAM_X_M + m['tz'], -m['tx']])

    @staticmethod
    def _rel(p, goal, phi):
        """목표 기준 로봇: (옆 m, 덜 들어간 거리 m (+면 벽에서 더 멀어져야), 방향 rad)"""
        n = np.array([math.cos(phi), math.sin(phi)])          # 벽 -> 칸 안쪽
        d = np.array(p[:2]) - np.array(goal[:2])
        return float(np.array([-n[1], n[0]]) @ d), float(-(n @ d)), md.wrap(p[2] - goal[2])

    def face(self, ref, tag, find=False):
        """기록 자세의 방향으로 (제자리 회전만). 가까우면 벽 기울기, 멀면 방위각"""
        for k in range(1, 4):
            m = self.see(ref['id'], ref.get('size', 0.025), find=find and k == 1)
            if m is None:
                raise RuntimeError(f'마커 {ref["id"]}가 안 보임 ({tag})')
            near = 'yaw' in ref and m['tz'] < YAW_OK_Z * ref.get('size', 0.025) / 0.025
            err = (m['yaw'] - ref['yaw']) if near else (bearing(m) - ref['bearing'])
            self.log(f'[{tag}] 방향 오차 {math.degrees(err):+.2f}° ({"기울기" if near else "방위각"}, '
                     f'마커 {m["tz"] * 100:.1f} cm)')
            if abs(math.degrees(err)) > MAX_DEG:
                raise RuntimeError(f'{tag}: 방향 오차 {math.degrees(err):+.0f}° (시작 자세 · 기록 확인)')
            if abs(math.degrees(err)) < TOL_DEG:
                return
            self.mv.dk.rotate_to(md.wrap(self.mv.dk.pose()[2] + err))
            if abs(math.degrees(err)) < VERIFY_DEG:
                return

    def reach(self, ref, tag, vmax=V_FAR):
        """마커 거리(tz)를 기록값까지 지금 방향 그대로 앞뒤로"""
        for k in range(1, 4):
            m = self.see(ref['id'], ref.get('size', 0.025))
            if m is None:
                raise RuntimeError(f'마커 {ref["id"]}가 안 보임 ({tag})')
            d = m['tz'] - ref['tz']
            self.log(f'[{tag}] 거리 {m["tz"] * 100:.1f} cm (목표 {ref["tz"] * 100:.1f}) -> '
                     f'{"앞" if d > 0 else "뒤"}로 {abs(d) * 100:.1f} cm')
            if abs(d) < FIX_M:
                return
            self.mv.drive(abs(d), d < 0, self.mv.dk.pose()[2], vmax if k == 1 else V_NEAR)

    def turn(self, deg, tag):
        e = self.mv.dk.rotate_to(md.wrap(self.mv.dk.pose()[2] + math.radians(deg)))
        self.log(f'[{tag}] {deg:+.0f}° 회전 (남은 {e:+.1f}°)')

    def go_to(self, x, y, vmax=V_NEAR):
        """(x, y)까지 곧게. 뒤쪽이면 후진 (앞이 마커를 보는 채로), 앞쪽이면 전진. 180° 돌지 않음"""
        p = self.mv.dk.pose()
        dx, dy = x - p[0], y - p[1]
        dist = math.hypot(dx, dy)
        if dist < 0.003:
            return
        back = dx * math.cos(p[2]) + dy * math.sin(p[2]) < 0
        h = math.atan2(-dy, -dx) if back else math.atan2(dy, dx)
        if abs(math.degrees(md.wrap(h - p[2]))) > 1.0:
            self.mv.dk.rotate_to(h)
        self.mv.drive(dist, back, h, vmax)

    def match(self, marker, rec, tag):
        """N15: 도크 마커를 가까이(15 cm) 보고 기록 자세(rec)로. 앞뒤 · 방향만 확실히 맞추고,
        옆(2 cm 안)은 후진하면서 줄 따라가기가 고치니 여기서 억지로 옆걸음하지 않음 (흔들림 방지)
        반환 (D, phi): 마커 위치 · 벽 방향 (15장 중앙값, 가장 정확한 측정)"""
        size = rec.get('size', 0.025)
        D = phi = None
        for k in range(1, 4):
            m = self.see(marker, size, frames=15, find=(k == 1))
            if m is None:
                raise RuntimeError(f'마커 {marker}가 안 보임 ({tag})')
            p, D = self._marker_odom(m)
            phi = md.dock_in_odom(m, p)[1] if m['tz'] < YAW_OK_Z else md.wrap(p[2] + math.pi)
            T = md.dock_to_odom(D, phi, rec['gx'], rec['gy'], rec['gth'])
            lat, along, dth = self._rel(p, T, phi)
            self.log(f'[{tag} {k}] 마커 {m["tz"] * 100:.1f} cm: 옆 {lat * 1000:+.0f} mm, 앞뒤 {along * 1000:+.0f} mm, '
                     f'방향 {math.degrees(dth):+.1f}°')
            moved = False
            if abs(lat) > MATCH_LAT_M:
                self.go_to(T[0], T[1])
                moved = True
            elif abs(along) > FIX_M:
                self.mv.move_along(-along, p[2])
                moved = True
            if abs(math.degrees(md.wrap(T[2] - self.mv.dk.pose()[2]))) > MATCH_DEG or moved:
                self.mv.dk.rotate_to(T[2])
                moved = True
            if not moved:
                break
        return D, phi

    def park(self, marker, stop_rec, target_rec, name='N15'):
        """후면 주차 (실무 도킹 방식: 움직이면서 계속 보고, 정답 자세와 비교하며 줄을 따라감)
        - 정답 = D3 기록 자세. 벽 방향은 N15에서 멈춰서 잰 값(가장 정확)으로 고정
        - 후진 중 매 프레임 마커 위치를 재서 도크 위치를 조금씩 고침 (가까울수록 많이 믿음)
        - 줄에서 옆으로 벗어난 만큼만 아주 살짝 기움 (최대 PARK_STEER_DEG, 회전 속도 PARK_W_MAX 제한) -> 크게 안 틈
        - 끝: 멈춰서 정답 자세와 비교, 앞뒤 · 방향 마무리"""
        import time
        D, phi = self.match(marker, stop_rec, f'{name} 맞추기')
        g = target_rec
        size = g.get('size', 0.025)
        _set_size(size)
        self.cam.id = marker
        line_h = md.wrap(phi + math.pi)
        goal = md.dock_to_odom(D, phi, g['gx'], g['gy'], g['gth'])
        p = self.mv.dk.pose()
        self.log(f'[주차] 도크까지 {self._rel(p, goal, phi)[1] * 100:.1f} cm 후진 (줄 따라가기)')
        t0 = last = time.time()
        v_prev, n_seen, k = 0.0, 0, 0
        while time.time() - t0 < 25.0:
            tick = time.time()
            m = self.cam.measure()
            p = self.mv.dk.pose()
            if m is not None:
                Dm = np.array(p[:2]) + md.rot(p[2]) @ np.array([md.CAM_X_M + m['tz'], -m['tx']])
                a = PARK_ALPHA * min(1.0, (0.15 / max(m['tz'], 0.15)) ** 2)
                D = D + a * (Dm - D)
                goal = md.dock_to_odom(D, phi, g['gx'], g['gy'], g['gth'])
                last, n_seen = tick, n_seen + 1
            elif tick - last > 1.0:
                self.mv.stop(0.1)
                raise RuntimeError('주차 중 마커를 1초 넘게 못 봄')
            lat, along, _ = self._rel(p, goal, phi)
            if along < 0.002:
                break
            corr = -math.atan(PARK_K_LAT * lat)
            corr = max(-math.radians(PARK_STEER_DEG), min(math.radians(PARK_STEER_DEG), corr))
            w = max(-PARK_W_MAX, min(PARK_W_MAX, PARK_K_HEAD * md.wrap(line_h + corr - p[2])))
            v = min(V_PARK, 0.012 + 0.8 * along, v_prev + PARK_ACC / PARK_HZ)
            v_prev = v
            self.mv.send(-v, w)
            k += 1
            if k % PARK_HZ == 0:
                self.log(f'[주차 중] 남은 {along * 100:.1f} cm, 옆 {lat * 1000:+.0f} mm, '
                         f'방향 {math.degrees(md.wrap(p[2] - line_h)):+.1f}°')
            time.sleep(max(0.0, 1.0 / PARK_HZ - (time.time() - tick)))
        self.mv.stop(0.1)
        self.log(f'[주차] 후진 끝 ({time.time() - t0:.1f}초, 마커 {n_seen}번 봄)')
        # 끝: 멈춰서 정답과 비교 -> 방향 · 앞뒤
        self.mv.dk.rotate_to(goal[2])
        for k in range(1, 3):
            m = self.see(marker, size, frames=10)
            if m is None:
                self.log('[주차 끝] 마커가 안 보여 확인 생략')
                return
            p, Dm = self._marker_odom(m)
            goal = md.dock_to_odom(Dm, phi, g['gx'], g['gy'], g['gth'])
            lat, along, dth = self._rel(p, goal, phi)
            self.log(f'[주차 끝 {k}] 정답과: 옆 {lat * 1000:+.0f} mm, 앞뒤 {along * 1000:+.0f} mm, '
                     f'방향 {math.degrees(dth):+.1f}° (마커 {m["tz"] * 100:.1f} cm)')
            if abs(along) < FIX_M:
                break
            self.mv.move_along(-along, p[2])
            self.mv.dk.rotate_to(goal[2])

    def _bar_hit(self, st):
        if self.mv.ir.get('n', 0) != st['n']:
            st['n'] = self.mv.ir['n']
            st['hit'] = st['hit'] + 1 if sum(self.mv.blacks()) == 3 else 0
        return st['hit'] >= 2

    def follow_walls(self, travel, tag, stop_early=FOLLOW_STOP_EARLY, stop_on_bar=True):
        """통로 후진 (기록 없음): 방향은 라이다로 잰 '양옆 벽과 평행', 좌우는 출발점에서 그은 직선 (odom).
        스캔마다 양옆 벽을 직선으로 맞춰 로봇이 벽과 몇 ° 틀어졌는지 재고 통로 방향을 고침 (벽 각도는 패널이
        옆으로 어긋나도 안 변함). 양쪽이 2° 넘게 다르면 그 스캔은 버림. 벽이 안 보이면 직전 방향 그대로.
        travel - stop_early 만큼 가거나 (stop_on_bar면) 가로선(검검검)을 보면 멈춤. 끝에서 벽과 평행하게 돎"""
        import time
        p0 = self.mv.dk.pose()
        S = np.array(p0[:2])
        h_c = p0[2]                                   # 벽과 평행한 '로봇 방향' (로봇은 남쪽을 봄)
        st = {'n': self.mv.ir.get('n', 0), 'hit': 0}
        goal = max(0.02, travel - stop_early)
        v_prev, k, nfix, nskip = 0.0, 0, 0, 0
        t0 = time.time()
        why = 'timeout'
        U = V = 0.0
        d_ref, bW = None, 0.0
        # 출발 전: 멈춘 채로 보이는 벽(N14에서는 동쪽 벽)으로 방향을 먼저 맞추고 출발
        ms = []
        t_end = time.time() + 1.5
        while time.time() < t_end and len(ms) < 6:
            q = self.mv.dk.pose()
            pts = self.lidar.take() if self.lidar else None
            if pts is not None:
                fl = side_lines(pts, along=WALL_ALONG_M)
                angs = [a for (d_, a, n_, e_) in fl.values() if n_ >= WALL_MIN_PTS and e_ < WALL_MAX_STD]
                if angs and not (len(angs) == 2 and abs(angs[0] - angs[1]) > math.radians(2.0)):
                    ms.append(md.wrap(q[2] + float(np.mean(angs))))
            time.sleep(0.03)
        if len(ms) >= 3:
            h_c = math.atan2(np.mean(np.sin(ms)), np.mean(np.cos(ms)))
            self.mv.dk.rotate_to(h_c)
            self.log(f'[{tag}] 출발 전 벽 기준으로 {math.degrees(md.wrap(h_c - p0[2])):+.1f}° 고침 ({len(ms)}번 잼)')
        else:
            self.log(f'[{tag}] 출발 자리에서 벽이 안 보여 마커 정렬 방향으로 출발')
        t0 = time.time()
        self.log(f'[{tag}] 벽과 평행하게 후진 {goal * 100:.0f} cm (8 cm/s, 라이다)')
        while time.time() - t0 < 30.0:
            tick = time.time()
            p = self.mv.dk.pose()
            pts = self.lidar.take() if self.lidar else None
            if pts is not None:
                fl = side_lines(pts, along=WALL_ALONG_M)
                angs = [a for (d_, a, n_, e_) in fl.values() if n_ >= WALL_MIN_PTS and e_ < WALL_MAX_STD]
                if len(angs) == 2 and abs(angs[0] - angs[1]) > math.radians(2.0):
                    nskip += 1
                elif angs:
                    meas = md.wrap(p[2] + float(np.mean(angs)))      # 벽과 평행한 로봇 방향 (odom)
                    h_c = md.wrap(h_c + WALL_GAIN * md.wrap(meas - h_c))
                    nfix += 1
                # 좌우: 서쪽 벽(남쪽을 보면 오른쪽)까지 거리로 odom 옆 밀림을 고침. 방향이 잡힌 뒤 처음 본 거리를 기준으로
                w_ = fl.get('오른쪽')
                if w_ and w_[2] >= WALL_MIN_PTS and w_[3] < WALL_MAX_STD and nfix >= 8 and U > 0.15:
                    if d_ref is None:
                        d_ref = w_[0] + V
                    bW += WALL_GAIN * ((d_ref - w_[0]) - V)
            mdir = h_c + math.pi                          # 후진 진행 방향 (북쪽)
            u = np.array([math.cos(mdir), math.sin(mdir)])
            n = np.array([-u[1], u[0]])
            U = float((np.array(p[:2]) - S) @ u)
            V = float((np.array(p[:2]) - S) @ n) + bW     # 출발 직선에서 옆 (+ = 진행 방향 왼쪽 = 서쪽)
            if stop_on_bar and self._bar_hit(st):
                why = 'bar'
                break
            if U >= goal:
                why = 'dist'
                break
            mot = max(-math.radians(LANE_STEER_DEG), min(math.radians(LANE_STEER_DEG), -math.atan(LANE_K * V)))
            h_t = md.wrap(h_c + mot)                      # 앞을 이쪽으로 -> 몸은 줄 쪽으로
            w = max(-LANE_W_MAX, min(LANE_W_MAX, 2.5 * md.wrap(h_t - p[2])))
            v = min(FOLLOW_V, v_prev + 0.25 / 15, 0.03 + 1.5 * (goal - U))
            v_prev = v
            self.mv.send(-v, w)
            k += 1
            if k % 15 == 0:
                self.log(f'[{tag}] {U * 100:.0f} cm, 벽 기준 방향 고침 {math.degrees(md.wrap(h_c - p0[2])):+.1f}°, '
                         f'출발선에서 옆 {V * 1000:+.0f} mm, 벽 맞춤 {nfix}번')
            time.sleep(max(0.0, 1 / 15 - (time.time() - tick)))
        self.mv.stop(0.05)
        if why != 'timeout':
            self.mv.dk.rotate_to(h_c)                     # 끝: 벽과 평행하게 (줄 쪽으로 기울였던 것 펴기)
        self.log(f'[{tag}] {U * 100:.1f} cm ({"가로선 봄" if why == "bar" else "거리 도착" if why == "dist" else "시간 초과"}), '
                 f'방향 고침 {math.degrees(md.wrap(h_c - p0[2])):+.1f}°, 옆 {V * 1000:+.0f} mm, 벽 맞춤 {nfix}번 (버림 {nskip})')
        if why == 'timeout':
            raise RuntimeError('통로 후진 시간 초과')
        return why

    def square(self, mid, size, tag):
        """마커 기울기로 벽과 수직이 되게 제자리 회전 (가까울 때만: 5 cm 마커 44 cm 안)"""
        m = None
        for k in range(4):
            m = self.see(mid, size, frames=15)
            if m is None:
                raise RuntimeError(f'마커 {mid}가 안 보임 ({tag})')
            if abs(math.degrees(m['yaw'])) < 1.0:
                break
            self.mv.dk.rotate_to(md.wrap(self.mv.dk.pose()[2] + m['yaw']))
        self.log(f'[{tag}] 벽과 수직: 기울기 {math.degrees(m["yaw"]):+.1f}°, 마커 {m["tz"] * 100:.1f} cm')
        return m

    def record_all(self, cfg, n15_tz, n14_tz, dock_only=False):
        """로봇이 도크(D3)에 반듯하게 있을 때 (손으로 넣는 건 이것 하나):
        1. 앞으로 곧게 나와 마커 3이 n15_tz인 곳 = N15 -> n15 저장, d3 = n15 + 나온 거리만큼 곧게 뒤
        2. 좌로 90° (남쪽) -> n15s 저장 (마커 5, 멀어서 방위각)
        3. 그 축 그대로 전진해 마커 5가 n14_tz인 곳 = N14 -> 벽과 수직 정렬 -> n14s 저장
        4. 좌로 90° (동쪽) -> 마커 4와 벽 수직 정렬 -> n14e 저장
        N14 · N15가 같은 통로 줄 위, N15가 칸 정면이 되게 로봇이 직접 맞추고 찍음"""
        marker = cfg['marker']
        m = self.see(marker, 0.025, frames=30)
        if m is None:
            raise RuntimeError(f'마커 {marker}가 안 보임 (도크 안에서 앞이 마커 {marker}를 보게)')
        self.log(f'[기록] 도크 자리: 마커 {m["tz"] * 100:.1f} cm, 좌우 {m["tx"] * 100:+.1f} cm')
        p0 = self.mv.dk.pose()
        h = p0[2]
        for k in range(3):
            d = m['tz'] - n15_tz
            if abs(d) < 0.003:
                break
            self.mv.drive(abs(d), d < 0, h, V_NEAR)
            m = self.see(marker, 0.025)
            if m is None:
                raise RuntimeError('나오는 중 마커가 안 보임')
        p1 = self.mv.dk.pose()
        s_out = (p1[0] - p0[0]) * math.cos(h) + (p1[1] - p0[1]) * math.sin(h)
        m = self.see(marker, 0.025, frames=30)
        g = md.goal_from_record(m)
        g.update(fx=md.FX, size=0.025)
        _save(f'~/dock_target_{cfg["stop"]}.json', g)
        gd = dict(g)
        gd['gx'] = g['gx'] - s_out * math.cos(g['gth'])
        gd['gy'] = g['gy'] - s_out * math.sin(g['gth'])
        _save(f'~/dock_target_{cfg["target"]}.json', gd)
        self.log(f'[기록] {cfg["stop"]}: 마커 {m["tz"] * 100:.1f} cm, 벽과 {math.degrees(md.wrap(g["gth"] - math.pi)):+.1f}° '
                 f'/ {cfg["target"]}: {s_out * 100:.1f} cm 뒤 (도크에 반듯하게 넣었으면 ±1° 안)')
        self.turn(90.0, '좌로 (남쪽)')
        m = self.see(5, 0.05, frames=30)
        if m is None:
            raise RuntimeError('N15에서 마커 5가 안 보임')
        _save(f'~/align_{cfg["via"]}.json', align_record(5, 0.05, m))
        self.log(f'[기록] {cfg["via"]}: 마커 5 거리 {m["tz"] * 100:.1f} cm, 방위각 {math.degrees(bearing(m)):+.2f}°')
        if dock_only:
            return f'기록 끝 ({cfg["target"]} · {cfg["stop"]} · {cfg["via"]})'
        self.reach({'id': 5, 'size': 0.05, 'tz': n14_tz}, 'N14로 전진')
        m = self.square(5, 0.05, 'N14 남쪽 벽')
        _save(f'~/align_{cfg["axis"]}.json', align_record(5, 0.05, m))
        self.log(f'[기록] {cfg["axis"]} 저장')
        self.turn(90.0, '좌로 (동쪽)')
        m = self.square(4, 0.05, 'N14 동쪽 벽')
        _save(f'~/align_{cfg["front"]}.json', align_record(4, 0.05, m))
        self.log(f'[기록] {cfg["front"]} 저장')
        return f'기록 끝 ({cfg["target"]} · {cfg["stop"]} · {cfg["via"]} · {cfg["axis"]} · {cfg["front"]})'

    def n14_align(self, front, axis, turn_deg=TURN_DEG):
        """N14 (Nav2로 도착, 동쪽 보기): 돌면서 마커 4 찾기 · 정렬 -> 기록 거리까지 앞뒤 -> 다시 정렬
        -> 우로 90° -> 마커 5와 벽면 수직.  front = align_n14e, axis = align_n14s 기록 (dict)"""
        self.face(front, 'N14 마커 찾기 · 정렬', find=True)
        self.reach(front, 'N14 거리')
        self.face(front, 'N14 정렬')
        self.turn(turn_deg, 'N14')
        self.face(axis, 'N14 축 정렬')

    def dock_in(self, cfg, travel=0.0):
        """통로에서 도크로 (D3 · D2): [travel > 0이면 벽과 평행하게 그만큼 후진] -> 마커 5 거리로 N15(N16) 앞뒤 마무리
        -> 우로 90° -> 마커 3(2)으로 N15 기록 자세에 맞추고 -> 후면 주차 (d3 기록)
        cfg: marker, stop (dock_target 이름), target (dock_target 이름), via (align 이름, 마커 5), turn_deg"""
        via = _load(f'~/align_{cfg["via"]}.json')
        name = str(cfg['stop']).upper()
        if travel > 0.03 and self.lidar is not None:
            self.follow_walls(travel, f'통로 -> {name}', stop_early=0.0, stop_on_bar=False)
        self.reach(via, f'{name} 거리', vmax=V_NEAR)
        self.turn(float(cfg.get('turn_deg', TURN_DEG)), name)
        self.park(int(cfg['marker']), _load(f'~/dock_target_{cfg["stop"]}.json'),
                  _load(f'~/dock_target_{cfg["target"]}.json'), name)

    def park_from_line(self, cfg):
        """도크 줄 가로선(N15 / N16) 위에서 남쪽을 보고 섬 -> 우90 -> 마커로 줄 기록 자세 맞춤 -> 후면 주차"""
        name = str(cfg['stop']).upper()
        self.turn(float(cfg.get('turn_deg', TURN_DEG)), name)
        self.park(int(cfg['marker']), _load(f'~/dock_target_{cfg["stop"]}.json'),
                  _load(f'~/dock_target_{cfg["target"]}.json'), name)

    def follow_bars(self, names, start, on_pass=None, resume=False):
        """통로 후진 (남쪽을 보고 북쪽으로): 방향 = 라이다 양옆 벽과 평행, 옆 = 서쪽 벽에서 W_REF.
        names() -> start 다음부터 지나갈 통로 노드 이름 (달리는 중에 늘어날 수 있음, 마지막 = 설 곳).
        가로선 노드는 선을 밟는 순간, 거리 노드(P2)는 거리로 on_pass(이름)을 부르고 멈추지 않음.
        반환 (마지막으로 지난 노드, 이유 bar / dist / p1_dist / none)"""
        import time
        ir = self.mv.ir
        p0 = self.mv.dk.pose()
        S = np.array(p0[:2])
        h_c = h0 = p0[2]
        ms, t_end = [], time.time() + 1.5
        while time.time() < t_end and len(ms) < 6:
            q = self.mv.dk.pose()
            pts = self.lidar.take() if self.lidar else None
            if pts is not None:
                fl = side_lines(pts, along=WALL_ALONG_M)
                angs = [a for (d_, a, n_, e_) in fl.values() if n_ >= WALL_MIN_PTS and e_ < WALL_MAX_STD]
                if angs and not (len(angs) == 2 and abs(angs[0] - angs[1]) > math.radians(2.0)):
                    ms.append(md.wrap(q[2] + float(np.mean(angs))))
            time.sleep(0.03)
        if len(ms) >= 3:
            h_c = h0 = math.atan2(np.mean(np.sin(ms)), np.mean(np.cos(ms)))
            self.mv.dk.rotate_to(h_c)
            self.log(f'[{start}] 출발 전 벽 기준으로 {math.degrees(md.wrap(h_c - p0[2])):+.1f}° 고침')
        else:
            self.log(f'[{start}] 출발 자리에서 벽이 안 보여 지금 방향으로 출발')
        on_start_bar = bool(ir.get('on'))
        bars_seen = ir.get('bar_n', 0)
        seq = list(names())
        self.log(f'[{start} -> ] 벽과 평행하게 후진: {seq}{" (출발 자리 선 위)" if on_start_bar else ""}')
        last, i, ref_U, fb_U = start, 0, 0.0, None
        U = V = bW = 0.0
        nfix = nskip = k = 0
        v_prev = 0.0
        why = 'none'
        t0 = time.time()
        while seq and time.time() - t0 < 60.0:
            tick = time.time()
            new = list(names())
            if new != seq:
                seq = seq[:i] + [n for n in new if n not in seq[:i]]
                self.log(f'  노드 늘어남: {seq[i:]}')
            stop = seq[-1]
            p = self.mv.dk.pose()
            pts = self.lidar.take() if self.lidar else None
            if pts is not None:
                fl = side_lines(pts, along=WALL_ALONG_M)
                angs = [a for (d_, a, n_, e_) in fl.values() if n_ >= WALL_MIN_PTS and e_ < WALL_MAX_STD]
                if len(angs) == 2 and abs(angs[0] - angs[1]) > math.radians(2.0):
                    nskip += 1
                elif angs:
                    h_c = md.wrap(h_c + WALL_GAIN * md.wrap(md.wrap(p[2] + float(np.mean(angs))) - h_c))
                    nfix += 1
                w_ = fl.get('오른쪽')                       # 남쪽을 보면 오른쪽 = 서쪽 칸막이
                if w_ and w_[2] >= WALL_MIN_PTS and w_[3] < WALL_MAX_STD and nfix >= 3:
                    bW += WALL_GAIN * ((W_REF - w_[0]) - V)
            mdir = h_c + math.pi
            u = np.array([math.cos(mdir), math.sin(mdir)])
            n = np.array([-u[1], u[0]])
            U = float((np.array(p[:2]) - S) @ u)
            V = float((np.array(p[:2]) - S) @ n) + bW
            # ---- 노드 판정
            new_bar = ir.get('bar_n', 0) > bars_seen
            bars_seen = ir.get('bar_n', 0)
            if new_bar and on_start_bar and U < START_GUARD_M:
                new_bar = False
            if new_bar and fb_U is not None and U - fb_U < RESYNC_M:
                self.log(f'  (늦게 나온 선: 거리로 친 {last}의 선, {(U - fb_U) * 100:.1f} cm 뒤)')
                ref_U, fb_U, new_bar = U, None, False
            done = False
            while i < len(seq):
                name = seq[i]
                kind, gap = BAR_KIND.get(name, 'dist'), BAR_GAP.get(name, 0.30)
                exp = ref_U + gap
                early_ok = resume and i == 0
                if new_bar and kind == 'bar' and U - ref_U < EARLY * gap and not early_ok:
                    self.log(f'  (너무 이른 선 무시: {name} 예상 {gap * 100:.0f} cm인데 {(U - ref_U) * 100:.1f} cm)')
                    new_bar = False
                if new_bar and kind == 'dist' and U < exp - 0.15:
                    self.log(f'  (너무 이른 선 무시: {name}까지 {(exp - U) * 100:.0f} cm 남음)')
                    new_bar = False
                if kind == 'bar':
                    if new_bar:
                        self.log(f'[{name} 도착] 가로선, {last}에서 {(U - ref_U) * 100:.1f} cm (예상 {gap * 100:.0f}), 옆 {V * 1000:+.0f} mm')
                        ref_U, fb_U, new_bar = U, None, False
                    elif U > exp + MISS_M and name != 'P1':
                        self.log(f'[{name} 도착] !! 선 못 봄 -> 거리로 ({(U - ref_U) * 100:.1f} cm)')
                        ref_U, fb_U = U, U
                    else:
                        break
                else:
                    if U >= exp or new_bar:
                        self.log(f'[{name} 도착] 거리 {(U - ref_U) * 100:.1f} cm, 옆 {V * 1000:+.0f} mm')
                        ref_U, fb_U = U, None
                    else:
                        break
                last = name
                i += 1
                if on_pass:
                    on_pass(name)
                if name == stop:
                    done = True
                    why = 'bar' if kind == 'bar' and fb_U is None else 'dist'
                    break
            if done:
                break
            if stop == 'P1' and i == len(seq) - 1 and U > ref_U + BAR_GAP['P1'] + MISS_M:
                why = 'p1_dist'                           # P1 선을 못 봄: tape가 더 찾음
                break
            nxt = seq[i]
            left = ref_U + BAR_GAP.get(nxt, 0.30) - U
            vmax = V_BAR_SLOW if (nxt == stop and (left < SLOW_M or nxt == 'P1')) else FOLLOW_V
            if nxt == stop and BAR_KIND.get(nxt, 'dist') == 'dist':
                vmax = min(vmax, 0.02 + 1.5 * max(0.0, left))
            mot = max(-math.radians(LANE_STEER_DEG), min(math.radians(LANE_STEER_DEG), -math.atan(LANE_K * V)))
            w = max(-LANE_W_MAX, min(LANE_W_MAX, 2.5 * md.wrap(md.wrap(h_c + mot) - p[2])))
            v = min(vmax, v_prev + 0.25 / 15)
            v_prev = v
            self.mv.send(-v, w)
            k += 1
            if k % 15 == 0:
                self.log(f'  {U * 100:.0f} cm, 벽 방향 고침 {math.degrees(md.wrap(h_c - h0)):+.1f}°, 옆 {V * 1000:+.0f} mm '
                         f'(벽 맞춤 {nfix}, 버림 {nskip})')
            time.sleep(max(0.0, 1 / 15 - (time.time() - tick)))
        else:
            if seq:
                self.mv.stop(0.1)
                raise RuntimeError('통로 후진 시간 초과')
        self.mv.stop(0.1)
        self.mv.dk.rotate_to(h_c)
        if why == 'bar' and sum(self.mv.blacks()) < 3:
            r, s_, _ = self.mv.drive(0.04, False, h_c, 0.02, event=lambda b, armed, s: sum(b) == 3)
            self.log(f'  선 지나쳐 앞으로 {s_ * 100:.1f} cm 돌아옴 ({"선 위" if r == "event" else "못 찾음"})')
        self.log(f'[{last}] 멈춤 ({why}): 출발에서 {U * 100:.1f} cm, 옆 {V * 1000:+.0f} mm, '
                 f'벽 방향 고침 {math.degrees(md.wrap(h_c - h0)):+.1f}°')
        return last, why


def check(aligns, docks):
    """기록 파일 요약 (rad -> cm · °). docks = (N15 자리 이름, 도크 이름)"""
    out = []
    for name in aligns:
        try:
            r = _load(f'~/align_{name}.json')
            out.append(f'align_{name}: 마커 {r["id"]}, 거리 {r["tz"] * 100:.1f} cm, 방위각 {math.degrees(r["bearing"]):+.2f}°, '
                       f'기울기 {math.degrees(r.get("yaw", 0)):+.1f}°')
        except Exception as e:
            out.append(f'align_{name}: 없음 ({e})')
    recs = {}
    for name in docks:
        try:
            r = recs[name] = _load(f'~/dock_target_{name}.json')
            out.append(f'dock_target_{name}: 앞 {r["gx"] * 100:.1f} cm, 옆 {r["gy"] * 100:+.1f} cm, '
                       f'방향 {math.degrees(md.wrap(r["gth"] - math.pi)):+.1f}° (0이면 벽과 수직)')
        except Exception as e:
            out.append(f'dock_target_{name}: 없음 ({e})')
    if len(recs) == 2:
        a, b = (recs[n] for n in docks)
        out.append(f'-> {docks[0]}와 {docks[1]}의 옆 차이 {(b["gy"] - a["gy"]) * 1000:+.0f} mm, '
                   f'방향 차이 {math.degrees(md.wrap(b["gth"] - a["gth"])):+.1f}° (둘 다 0이어야 곧게 후진해서 들어감)')
    return '\n'.join(out)

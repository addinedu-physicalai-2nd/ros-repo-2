"""테이블 정밀 정차 (T1~T6): 다리 마커로 차선 정렬 -> 바닥 정지선에서 정지,  서빙 뒤 빼기

docking 노드가 부른다 (precision_stops.yaml T 설정)
  TableDocker(node, pub, cancelled, tf_buf, cam, ir).run(req)     method: table       ('성공'이면 성공)
  TableDocker(node, pub, cancelled, tf_buf, None, ir).leave(req)   method: table_exit
  req = precision_stops 값 + node + from_node (Driver가 넣음: 테이블 직전에 선 노드)

table (run)
  Driver가 Nav2로 직전 노드(from_node)에 '들어가는 방향'으로 세운 뒤 맡긴다.
  1. 마커 찾기: 지금 방향 -> 기록에서 계산한 방향(차선 방향으로 섰을 때 마커 쪽) -> 그 기준 ±10 · ±20°
  2. 마커가 화면 가운데서 center_tol 넘게 벗어나 있으면 돌려서 가운데로 다시 잼 (기록 때와 같은 조건)
  3. 기록(dock_target_<target>.json, 마커 기준 차선)을 지금 odom으로 -> 차선에서 옆 · 방향 오차
  4. 차선 올라타기: 차선 위 entry_ahead 앞 점으로 바라보기 -> 직진 -> 차선 방향 (이미 차선 위면 생략)
     올라탄 뒤 마커가 아직 보이면 한 번 더 재서 차선을 고침
  5. 차선 방향 직진, 정렬 시작(4 직전)부터 센 lines번째 바닥 선에서 정지 (+ line_offset)
  선 세기: 직진할 때만 (회전 중 선 가장자리 무시), 센서 3개 중 ir_min_dark개 이상 검정이 confirm번 연속이면 선,
           선 사이로 보려면 흰색으로 min_white 이상. 시작할 때 선 위면 그 선을 #1로 (노드는 선 앞이라는 전제)

table_exit (leave)
  서빙 뒤 다음 출발 전에: 지금 방향으로 dist(m) 전진 (line_dock.Motion.drive) -> 그다음 Nav2

기록 파일: 팀 marker_dock과 같은 형식 {gx, gy, gth, fx, size} (도크 좌표계: 원점 = 마커, x = 법선)
  table_align.py --record-lane 으로 만든다 (차선 위 한 점 + 방향. gx 쪽 앞뒤 위치는 의미 없음)
  마커 ID = 테이블 번호 + 1(왼쪽) / 2(오른쪽), 이름 t1l · t1r … (fleet_msgs/docs/record_json_format.md)
"""
import json
import math
import os
import time

import numpy as np

from . import marker_dock as md
from .line_dock import DEFAULTS as LINE_DEFAULTS, Motion
from .undock import _set_size

DEFAULTS = {
    'lines': 2,              # 몇 번째 선에서 정지
    'entry_ahead': 0.08,     # 차선 올라타기 앞 거리 (m)
    'line_max': 0.40,        # 정렬 끝에서 선을 찾을 최대 거리 (m)
    'line_offset': 0.0,      # 선 감지 뒤 더 갈 거리 (m)
    'v_line': 0.04,          # 선 찾으며 직진 속도 (m/s)
    'ir_black': 3000,        # 이보다 크면 검정 (흰 바닥 ~240, 테이프 ~4095)
    'ir_min_dark': 2,        # 3개 중 이만큼 검정이면 선 (비스듬히 지날 때도 잡히게)
    'confirm': 2,            # 새 센서 값이 연속 이만큼 맞아야 검정/흰색 인정
    'min_white': 0.01,       # 선 사이로 보려면 흰색으로 이만큼 (m)
    'frames': 15,            # 측정 장수 (중앙값)
    'center_tol_deg': 4.0,   # 마커가 가운데서 이보다 벗어나면 돌려서 다시 잼
    'on_lane_lat': 0.005,    # 이 안이면 이미 차선 위 (m)
    'on_lane_yaw_deg': 2.0,
    'find_steps_deg': [0, 10, -10, 20, -20],
    'spread_warn_deg': 3.0,  # 측정 흔들림 경고
    'dist': 0.05,            # table_exit: 서빙 뒤 전진 거리 (m)
    'v_exit': 0.04,
}


def lane_offset(pose, goal):
    """goal 자세가 정하는 차선(goal을 지나 goal 방향 직선)에서: (남은 거리, 옆 + 왼쪽, 방향 오차)"""
    rel = md.rot(-goal[2]) @ (np.array(pose[:2]) - np.array(goal[:2]))
    return float(-rel[0]), float(rel[1]), md.wrap(pose[2] - goal[2])


def lane_face_bearing(g):
    """기록 자세에 선 로봇에서 본 마커 방향 (+ 왼쪽) = 차선 방향으로 섰을 때 마커를 보려면 돌 각"""
    rel = md.rot(-g['gth']) @ (-np.array([g['gx'], g['gy']]))
    return math.atan2(rel[1], rel[0])


def bearing(m):
    return math.atan2(-m['tx'], md.CAM_X_M + m['tz'])


def default_name(marker_id):
    t, side = divmod(int(marker_id), 10)
    if 1 <= t <= 6 and side in (1, 2):
        return f't{t}{"l" if side == 1 else "r"}'
    return f'id{marker_id}'


class LineCounter:
    """바닥 선 세기 (ir = docking 노드의 {'v': [3개], 'n': 받은 횟수})"""

    def __init__(self, ir, cfg):
        self.ir = ir
        self.c = cfg
        self.on = False
        self.count = 0
        self.black = False
        self.b_streak = self.w_streak = 0
        self.last_n = -1
        self.in_line = False
        self.white_from = None
        self.starts = []

    def _sample(self):
        """새 센서 값이 왔으면 검정/흰색 판정 갱신"""
        n = self.ir.get('n', 0)
        v = self.ir.get('v')
        if v is None or n == self.last_n:
            return
        self.last_n = n
        dark = sum(int(x > self.c['ir_black']) for x in v[:3])
        if dark >= self.c['ir_min_dark']:
            self.b_streak += 1
            self.w_streak = 0
        elif dark == 0:
            self.w_streak += 1
            self.b_streak = 0
        if self.b_streak >= self.c['confirm']:
            self.black = True
        elif self.w_streak >= self.c['confirm']:
            self.black = False

    def start(self, p):
        self._sample()
        self.on = True
        self.count = 1 if self.black else 0
        self.in_line = self.black
        self.white_from = None if self.black else p[:2]
        if self.black:
            self.starts.append(0.0)
            md.log('[선] 선 위에서 시작 -> 이 선을 #1로 셈')

    def tick(self, p, s=0.0):
        """직진 중에만 부름. 새 선이면 True"""
        if not self.on:
            return False
        self._sample()
        if self.black:
            new = (not self.in_line and self.white_from is not None and
                   math.hypot(p[0] - self.white_from[0], p[1] - self.white_from[1]) >= self.c['min_white'])
            self.in_line = True
            if new:
                self.count += 1
                self.starts.append(s)
                md.log(f'[선] #{self.count} 감지')
            return new
        if self.in_line or self.white_from is None:
            self.white_from = p[:2]
        self.in_line = False
        return False


class TableDocker:
    def __init__(self, node, pub, cancelled, tf_buf, cam, ir):
        self.node = node
        self.pub = pub
        self.cancelled = cancelled
        self.tf_buf = tf_buf
        self.cam = cam
        self.ir = ir if ir is not None else {'v': None, 'n': 0}

    # ------------------------------------------------------------ 준비
    def _cfg(self, req):
        c = dict(DEFAULTS)
        c.update({k: req[k] for k in DEFAULTS if k in req})
        return c

    def _wait_ir(self, timeout=2.0):
        end = time.time() + timeout
        while self.ir.get('v') is None:
            if self.cancelled():
                raise md.Cancelled()
            if time.time() > end:
                raise RuntimeError('바닥 센서(/ir_sensor/range)가 안 들어옴. pinky_sensor_adc main_node 켰나요?')
            time.sleep(0.05)

    def _load_target(self, f):
        name = f.get('target') or default_name(f['marker'])
        path = os.path.expanduser(f'~/dock_target_{name}.json')
        if not os.path.exists(path):
            old = os.path.expanduser(f'~/table_target_id{int(f["marker"])}.json')   # table_align 예전 이름
            if os.path.exists(old):
                md.log(f'주의: {path} 없음 -> 예전 이름 {old} 사용')
                path = old
            else:
                raise RuntimeError(f'기록 없음 {path} (table_align.py --record-lane --id {f["marker"]})')
        with open(path) as fp:
            g = json.load(fp)
        if abs(g.get('fx', md.FX) - md.FX) > 1:
            md.log(f'주의: 기록 FX {g.get("fx")} != 지금 FX {md.FX:.1f} (보정 뒤 record 다시 권장)')
        return g, path

    # ------------------------------------------------------------ 측정
    def _measure(self, dk, tag, c, frames=None, timeout=4.0):
        dk.stop(md.MEASURE_SETTLE_S)
        m = self.cam.measure_stable(frames=frames or c['frames'], timeout=timeout)
        self.cam.snapshot(tag)
        if m is None:
            return None
        p = dk.wait_pose()
        D, phi = md.dock_in_odom(m, p)
        spread = math.degrees(m['yaw_spread'])
        md.log(f'[{tag}] 마커 앞 {m["tz"] * 100:.1f} cm, 좌우 {m["tx"] * 100:+.1f} cm, '
               f'각도 {math.degrees(m["yaw"]):+.1f}° (흔들림 {spread:.1f}°, {m["n"]}회), '
               f'가운데서 {math.degrees(bearing(m)):+.1f}°')
        if spread > c['spread_warn_deg']:
            md.log(f'  주의: 각도 흔들림 {spread:.1f}° (마커 크기 · 조명 확인)')
        return m, p, D, phi

    def _find(self, dk, c, face):
        p0 = dk.pose()[2]
        tries = [0.0]
        for st in c['find_steps_deg']:
            a = math.degrees(face) + st
            if all(abs(a - t) > 3.0 for t in tries):
                tries.append(a)
        for a in tries:
            if a != 0.0:
                dk.rotate_to(md.wrap(p0 + math.radians(a)))
            r = self._measure(dk, '측정' if a == 0.0 else f'측정 ({a:+.0f}°)', c)
            if r is not None:
                return self._center(dk, c, r)
        return None

    def _center(self, dk, c, r):
        for _ in range(2):
            ang = bearing(r[0])
            if abs(math.degrees(ang)) < c['center_tol_deg']:
                return r
            md.log(f'[가운데 맞추기] {math.degrees(ang):+.1f}° 벗어남 -> 돌려서 다시 잼')
            dk.rotate_to(md.wrap(r[1][2] + ang))
            r2 = self._measure(dk, '측정 (가운데)', c)
            if r2 is None:
                return r
            r = r2
        return r

    # ------------------------------------------------------------ 이동 (선 세며)
    def _drive(self, dk, lc, x, y, vmax, stop_on=None):
        """(x, y)까지 직진하며 선 세기 (Docker.drive_to + 선). stop_on개 세면 그 자리에서 정지. 반환: 선에서 멈췄나"""
        p0 = dk.pose()
        line = math.atan2(y - p0[1], x - p0[0])
        u = np.array([math.cos(line), math.sin(line)])
        t0, v = time.time(), 0.0
        hit = False
        while time.time() - t0 < md.MOVE_TIMEOUT_S:
            p = dk.pose()
            s = float(u @ (np.array(p[:2]) - np.array(p0[:2])))
            lc.tick(p, s)
            if stop_on is not None and lc.count >= stop_on:
                hit = True
                break
            remain = float(u @ (np.array([x, y]) - np.array(p[:2])))
            if remain < max(md.LIN_TOL_M, v * md.STOP_LEAD_S):
                break
            v = md.clamp(md.LIN_K * remain, md.LIN_V_MIN, vmax)
            dk.send(v, md.clamp(md.HEADING_K * md.wrap(line - p[2]), -0.4, 0.4))
            time.sleep(1 / md.MOVE_HZ)
        dk.stop()
        return hit

    def _move(self, dk, lc, x, y, th, tag):
        """odom 자세로: 바라보기 -> (선 세며) 직진 -> 방향 맞추기"""
        p = dk.pose()
        dist = math.hypot(x - p[0], y - p[1])
        if dist > 0.004:
            face = math.atan2(y - p[1], x - p[0])
            if abs(math.degrees(md.wrap(face - p[2]))) > md.FACE_SKIP_DEG:
                dk.rotate_to(face)
            self._drive(dk, lc, x, y, md.LIN_V_MAX)
            md.log(f'[{tag}] 직진 {dist * 100:.1f} cm')
        e = dk.rotate_to(th)
        md.log(f'[{tag}] 방향 맞추기 오차 {e:+.1f}°')

    # ------------------------------------------------------------ table
    def run(self, req):
        c = self._cfg(req)
        node_id, src = req.get('node', '?'), req.get('from_node')
        f = (req.get('from') or {}).get(src)
        if f is None:
            return f'실패: {node_id} from에 {src} 없음'
        md.load_cam_file()
        g, path = self._load_target(f)
        old_size = md.MARKER_SIZE_M
        _set_size(float(g.get('size', 0.025)))
        try:
            return self._run(req, c, f, g, path)
        finally:
            _set_size(old_size)          # 다른 도킹(marker_dock 기본 2.5 cm)에 영향 없게

    def _run(self, req, c, f, g, path):
        self.cam.id = int(f['marker'])
        dk = md.Docker(self.node, self.pub, self.tf_buf, self.cam, moving=True, spin=False,
                       cancelled=self.cancelled)
        dk.wait_pose()
        self._wait_ir()
        lc = LineCounter(self.ir, c)
        face = lane_face_bearing(g)
        md.log(f'[table] {req.get("node")} <- {req.get("from_node")}: 마커 {f["marker"]}, 기록 {path}, '
               f'{c["lines"]}번째 선에서 정지. 차선 방향이면 마커는 {math.degrees(face):+.1f}° 쪽')

        # 1~2. 찾기 · 가운데 · 측정
        r = self._find(dk, c, face)
        if r is None:
            return f'실패: 마커 {f["marker"]}가 안 보임'
        m, p, D, phi = r

        # 3. 차선
        goal = md.dock_to_odom(D, phi, g['gx'], g['gy'], g['gth'])
        _, lat, dth = lane_offset(p, goal)
        md.log(f'[판단] 차선에서 옆 {lat * 100:+.1f} cm, 방향 {math.degrees(dth):+.1f}° (측정 자세라 틀어진 게 정상)')

        # 4. 올라타기 (여기부터 선 셈)
        lc.start(p)
        if abs(lat) < c['on_lane_lat'] and abs(math.degrees(dth)) < c['on_lane_yaw_deg']:
            dk.rotate_to(goal[2])
        else:
            remain, _, _ = lane_offset(dk.pose(), goal)
            u = np.array([math.cos(goal[2]), math.sin(goal[2])])
            e = np.array(goal[:2]) - u * (remain - c['entry_ahead'])
            md.log(f'[올라타기] 차선 위 {c["entry_ahead"] * 100:.0f} cm 앞 점, '
                   f'비스듬히 약 {abs(math.degrees(math.atan2(lat, c["entry_ahead"]))):.0f}°')
            self._move(dk, lc, float(e[0]), float(e[1]), goal[2], '올라타기')
            r2 = self._measure(dk, '올라탄 뒤 재측정', c, frames=8, timeout=1.5)
            if r2 is not None:
                goal = md.dock_to_odom(r2[2], r2[3], g['gx'], g['gy'], g['gth'])
                dk.rotate_to(goal[2])
        p = dk.pose()
        _, lat, dth = lane_offset(p, goal)
        md.log(f'[차선 위] 옆 {lat * 1000:+.1f} mm, 방향 {math.degrees(dth):+.2f}°, 정렬 중 지난 선 {lc.count}개')

        # 5. 정지선
        if lc.count >= c['lines']:
            return f'실패: 정렬 중에 이미 선 {lc.count}개 지남 (entry_ahead 줄이기 / 노드를 선에서 떨어뜨리기)'
        x0, y0 = p[:2]
        far = (x0 + c['line_max'] * math.cos(goal[2]), y0 + c['line_max'] * math.sin(goal[2]))
        if not self._drive(dk, lc, *far, c['v_line'], stop_on=c['lines']):
            return f'실패: {c["line_max"] * 100:.0f} cm 가도 {c["lines"]}번째 선 못 찾음 (센 선 {lc.count}개)'
        if c['line_offset'] > 0:
            lc.on = False
            q = dk.pose()
            dk.drive_to(q[0] + c['line_offset'] * math.cos(goal[2]), q[1] + c['line_offset'] * math.sin(goal[2]))
        p = dk.pose()
        _, lat, dth = lane_offset(p, goal)
        md.log(f'[정지선] {c["lines"]}번째 선에서 정지: 차선에서 옆 {lat * 1000:+.1f} mm, 방향 {math.degrees(dth):+.2f}° '
               f'(정렬 끝에서 {math.hypot(p[0] - x0, p[1] - y0) * 100:.1f} cm)')
        return '성공'

    # ------------------------------------------------------------ table_exit
    def leave(self, req):
        c = self._cfg(req)
        md.open_log(f'{req.get("node")}_table_exit')
        mv = Motion(self.node, self.pub, self.cancelled, self.tf_buf, self.ir, dict(LINE_DEFAULTS))
        mv.dk.wait_pose()
        h = mv.dk.pose()[2]
        why, s, _ = mv.drive(c['dist'], False, h, c['v_exit'])
        md.log(f'[table_exit] {s * 100:.1f} cm 전진 ({why}) -> 그다음 Nav2')
        return '성공' if why == 'done' else f'실패: 빼기 {why} ({s * 100:.1f} cm)'


__all__ = ['TableDocker', 'LineCounter', 'lane_face_bearing', 'lane_offset']

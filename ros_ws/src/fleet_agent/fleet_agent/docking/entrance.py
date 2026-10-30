"""입구 통로 D1 도킹 · 빠지기 (안내 로봇 자리). docking 노드 method entrance / leave entrance_exit, 기록은 dock_record d1

교점 = D1에서 곧게 앞으로 나간 줄(마커 1 줄)과, 남쪽을 봤을 때 마커 6이 정면인 줄이 만나는 곳
서쪽을 볼 때 방향 = 앞 벽(서쪽 벽)을 라이다로 직선 맞춤 (odom 90° 회전은 몇 ° 틀림 -> 마커를 정면에 둬도 2 cm 비껴 섬)
남쪽 · 북쪽을 볼 때 방향 = 양옆 벽(서쪽 벽 · 칸막이)과 평행, 옆 = 두 벽 가운데

  record (D1에 손으로 반듯하게, 앞 = 마커 1, 바구니 = 칸막이):
        D1 기록 -> 곧게 앞으로 -> 좌90 마커 6 정면 맞춤(교점) -> 우90 교점 기록(마커 1) -> 좌90 교점 기록(마커 6 · 벽)
        -> 가운데로 북쪽 후진, 칸막이 끝 = N1 기록. 끝: N1에서 남쪽을 봄
  dock   (N1에서 D1 쪽을 보고): 가운데로 칸막이 시작 + 기록 거리 -> 마커 6 거리 -> 우90 벽 수직
        -> 마커 1 = 교점 기록 -> 곧게 후진, 마커 1 거리 = D1 기록
  undock (D1): 마커 1 거리로 곧게 앞으로 교점 -> [우로 450° 이어서 회전 = 우90 + 한 바퀴, AMCL] -> 통로 벽과 평행 북쪽
        (그다음 fleet_agent가 Nav2로 N1). 마커 6 · 남쪽은 안 봄 (그건 dock / record)
기록 파일: ~/align_d1s.json (주로) · ~/dock_target_d1.json · ~/dock_target_d1line.json
"""
import math
import time

import numpy as np

from . import marker_dock as md
from .corridor import WALL_ALONG_M, WALL_GAIN, WALL_MAX_STD, WALL_MIN_PTS, Steps, _load, _save, side_lines

LINE_TZ = 0.21         # 기록: D1에서 곧게 앞으로 교점까지 대략 (마커 1 거리 m, 마커 6으로 맞춤)
V_GO = 0.08
LAT_K = 3.0            # 서쪽 벽 거리 1 cm 틀리면 1.7° 기울여 감
LAT_DEG = 6.0
W_MAX = 0.25
CLOSE_TZ = 0.15        # 마커 1을 이 거리에서 재서 벽 방향을 정확히 (2.5 cm 마커는 22 cm 안에서만 기울기를 믿음)
END_GONE_M = 0.05      # 칸막이가 이만큼 이어서 안 보이면 끝 (N1)
END_MIN_M = 0.40       # 이보다 가까운 끊김은 무시 (칸막이 이음새)
END_MAX_M = 1.60
SIDE_FAR = 0.60        # 통로 벽 찾는 범위 (corridor 기본 0.45보다 넓게: 입구 통로가 더 넓음)


def lines(pts):
    return side_lines(pts, along=WALL_ALONG_M, far=SIDE_FAR)


def side_here(pts, side):
    """로봇 중심 바로 옆(앞뒤 4 cm 안)에 그쪽 벽 점이 있나 -> 칸막이 끝 · 시작을 로봇 중심 기준으로 (직선 맞춤은 앞뒤 25 cm를 봐서 늦음)"""
    sg = 1 if side == '왼쪽' else -1
    m = (np.abs(pts[:, 0]) < 0.04) & (sg * pts[:, 1] > 0.05) & (sg * pts[:, 1] < SIDE_FAR)
    return int(np.sum(m)) >= 3


def wall_scan(lidar, secs=1.5):
    """멈춘 채 양옆 벽: {side: (거리, 각, 횟수)} 중앙값"""
    acc = {'왼쪽': [], '오른쪽': []}
    t_end = time.time() + secs
    while time.time() < t_end:
        pts = lidar.take()
        if pts is not None:
            for side, (d_, a_, n_, e_) in lines(pts).items():
                if n_ >= WALL_MIN_PTS and e_ < WALL_MAX_STD:
                    acc[side].append((d_, a_))
        time.sleep(0.03)
    return {s: (float(np.median([x[0] for x in v])), float(np.median([x[1] for x in v])), len(v))
            for s, v in acc.items() if len(v) >= 3}


def front_wall(pts):
    """로봇 앞 벽(서쪽 벽, 마커 1 벽)을 직선으로: x = a + b*y (앞 5~60 cm, 옆 ±15 cm). 반환 (거리, 기울기 rad) 또는 None
    기울기 + = 로봇이 벽 수직보다 왼쪽(반시계)으로 돌아 있음"""
    m = (pts[:, 0] > 0.05) & (pts[:, 0] < 0.60) & (np.abs(pts[:, 1]) < 0.15)
    q = pts[m]
    for _ in range(3):
        if len(q) < 15:
            return None
        A = np.stack([np.ones(len(q)), q[:, 1]], axis=1)
        (a, b), *_ = np.linalg.lstsq(A, q[:, 0], rcond=None)
        res = q[:, 0] - (a + b * q[:, 1])
        keep = np.abs(res) < max(0.008, 2.5 * np.std(res))
        if keep.all():
            break
        q = q[keep]
    if np.std(res) > WALL_MAX_STD:
        return None
    return float(a), float(math.atan(b))


class Entrance(Steps):
    """입구 통로 D1: record (dock_record d1) · dock (method entrance) · undock (leave entrance_exit)"""

    def __init__(self, mv, cam, log, lidar=None, line_tz=LINE_TZ):
        super().__init__(mv, cam, log, lidar)
        self.line_tz = line_tz
        self.seen_at = None

    def front_square(self, tag):
        """앞 벽(서쪽)과 라이다로 수직 맞춤. odom 90° 회전은 몇 ° 틀림 (기록 때 +6.1°) -> 마커 1을 정면에 둬도
        로봇 중심이 통로 방향으로 (거리 x sin각) 만큼 비껴 섬 (사진: 연두색 칸보다 2 cm 북쪽). 반환 맞춘 방향"""
        for k in range(1, 4):
            acc, t_end = [], time.time() + 1.0
            while time.time() < t_end:
                pts = self.lidar.take()
                if pts is not None:
                    r = front_wall(pts)
                    if r is not None:
                        acc.append(r[1])
                time.sleep(0.03)
            if len(acc) < 3:
                self.log(f'[{tag}] 앞 벽이 라이다에 안 잡힘 -> 지금 방향 그대로')
                break
            th = float(np.median(acc))
            self.log(f'[{tag} {k}] 앞 벽과 {math.degrees(th):+.1f}° 틀어짐')
            if abs(math.degrees(th)) < 1.0:   # rotate_to가 0.8° 안은 안 돌아서 0.4°면 같은 값으로 3번 헛돎 (실측 3.5초)
                break
            self.mv.dk.rotate_to(md.wrap(self.mv.dk.pose()[2] - th))
        return self.mv.dk.pose()[2]

    def wall_heading(self, tag):
        """멈춘 채 양옆 벽과 평행하게 돎. 반환 (벽 기준 방향, 벽 거리들)"""
        ws = wall_scan(self.lidar)
        angs = [a for (d_, a, n_) in ws.values()]
        if not angs:
            self.log(f'[{tag}] 벽이 안 보임 -> 지금 방향 그대로')
            return self.mv.dk.pose()[2], ws
        if len(angs) == 2 and abs(angs[0] - angs[1]) > math.radians(2.0):
            angs = [ws['오른쪽'][1]] if '오른쪽' in ws else angs[:1]
        h = md.wrap(self.mv.dk.pose()[2] + float(np.mean(angs)))
        self.mv.dk.rotate_to(h)
        self.log(f'[{tag}] 벽과 평행: {math.degrees(float(np.mean(angs))):+.1f}° 고침, '
                 + ', '.join(f'{s} {d_ * 100:.1f} cm' for s, (d_, a_, n_) in ws.items()))
        return h, ws

    def go(self, dist, west, w_ref, tag, reverse=False, end_side=None, appear_side=None, after=None):
        """dist만큼 (reverse면 뒤로): 방향 = 벽 평행 (계속 고침), 옆 = 통로 가운데 (양쪽 벽이 보이면 두 거리 차,
        한쪽만 보이면 서쪽 벽(west = '왼쪽'/'오른쪽')까지 w_ref).
        end_side가 있으면 그쪽 벽이 END_GONE_M 이어서 안 보이는 곳(END_MIN_M 넘어서)에서 멈춤.
        appear_side가 있으면 그쪽 벽이 처음 나타난 곳에서 after만큼 더 가서 멈춤 (N1 -> 칸막이 시작 기준)
        반환 (벽 방향, 간 거리, 끝 찾음이면 끊긴 곳 거리 아니면 None)"""
        p0 = self.mv.dk.pose()
        S = np.array(p0[:2])
        h_c, h0 = p0[2], p0[2]
        sgn = -1.0 if reverse else 1.0
        y_err, nfix, U, v_prev, k = 0.0, 0, 0.0, 0.0, 0
        gone_at, end_at = None, None
        seen_at, seen_n, ref_done = None, 0, False
        t0 = time.time()
        while time.time() - t0 < 40.0:
            tick = time.time()
            p = self.mv.dk.pose()
            u = np.array([math.cos(h_c), math.sin(h_c)]) * sgn
            U = float((np.array(p[:2]) - S) @ u)
            pts = self.lidar.take()
            if pts is not None:
                fl = lines(pts)
                ok = {s_: v_ for s_, v_ in fl.items() if v_[2] >= WALL_MIN_PTS and v_[3] < WALL_MAX_STD}
                angs = [v_[1] for v_ in ok.values()]
                if angs and not (len(angs) == 2 and abs(angs[0] - angs[1]) > math.radians(2.0)):
                    h_c = md.wrap(h_c + WALL_GAIN * md.wrap(md.wrap(p[2] + float(np.mean(angs))) - h_c))
                    nfix += 1
                w_ = ok.get(west)
                if '왼쪽' in ok and '오른쪽' in ok:            # 양쪽 다 보이면 통로 가운데
                    e = (ok['오른쪽'][0] - ok['왼쪽'][0]) / 2.0     # + = 가운데보다 로봇 왼쪽에 있음
                    y_err += 0.5 * (e - y_err)
                elif w_ and w_ref:                              # 한쪽만: 서쪽 벽에서 가운데 거리
                    e = (w_ref - w_[0]) if west == '왼쪽' else (w_[0] - w_ref)
                    y_err += 0.5 * (e - y_err)
                if appear_side and not ref_done:
                    if side_here(pts, appear_side):
                        seen_at = U if seen_at is None else seen_at
                        seen_n += 1
                        if seen_n >= 3:
                            dist, ref_done = seen_at + after, True
                            self.log(f'  {appear_side} 벽 시작 {seen_at * 100:.1f} cm -> {dist * 100:.1f} cm까지')
                    else:
                        seen_at, seen_n = None, 0
                if end_side:
                    if side_here(pts, end_side) or U < END_MIN_M:
                        gone_at = None
                    elif gone_at is None:
                        gone_at = U
                    elif U - gone_at >= END_GONE_M:
                        end_at = gone_at
                        break
            if U >= dist:
                break
            c = math.atan(LAT_K * y_err)
            corr = max(-math.radians(LAT_DEG), min(math.radians(LAT_DEG), c if reverse else -c))
            w = max(-W_MAX, min(W_MAX, 2.5 * md.wrap(h_c + corr - p[2])))
            v = min(V_GO, v_prev + 0.25 / 15, 0.02 + 1.5 * max(0.0, dist - U))
            v_prev = v
            self.mv.send(sgn * v, w)
            k += 1
            if k % 15 == 0:
                self.log(f'  {U * 100:.0f} cm, 벽 방향 고침 {math.degrees(md.wrap(h_c - h0)):+.1f}°, 줄에서 옆 {y_err * 1000:+.0f} mm')
            time.sleep(max(0.0, 1 / 15 - (time.time() - tick)))
        self.mv.stop(0.1)
        self.mv.dk.rotate_to(h_c)
        self.log(f'[{tag}] {U * 100:.1f} cm {"후진" if reverse else "전진"}, 벽 방향 고침 {math.degrees(md.wrap(h_c - h0)):+.1f}°, '
                 f'줄에서 옆 {y_err * 1000:+.0f} mm (벽 맞춤 {nfix}번)'
                 + (f', {end_side} 벽 끝 {end_at * 100:.1f} cm' if end_at is not None else ''))
        self.seen_at = seen_at if ref_done else None
        return h_c, U, end_at

    def along6(self, h, tag):
        """마커 6(남쪽 끝 벽)까지 통로 방향 거리. 비껴 있으면 돌아서 찾고 다시 h로. 없으면 None"""
        m = self.see(6, 0.05, frames=15, find=True)
        if m is None:
            self.mv.dk.rotate_to(h)
            return None
        p, M = self._marker_odom(m)
        u = np.array([math.cos(h), math.sin(h)])
        along = float((M - np.array(p[:2])) @ u)
        self.mv.dk.rotate_to(h)
        self.log(f'[{tag}] 마커 6: 통로 방향 {along * 100:.1f} cm (마커 {m["tz"] * 100:.1f} cm)')
        return along

    def see_near(self, mid, h, size=0.025):
        """정면부터 ±10° · ±20°만 돌며 찾음 (30° 크게 돌아 찾은 먼 마커로 방향을 잡으면 옆으로 벌어짐). 끝나면 h로"""
        m = self.see(mid, size, frames=15)
        for off in (10, -10, 20, -20):
            if m is not None:
                break
            self.mv.dk.rotate_to(md.wrap(h + math.radians(off)))
            m = self.see(mid, size, frames=15)
            if m is not None:
                self.log(f'  마커 {mid}: {off:+d}°에서 찾음')
        if m is not None:
            m = dict(m)
            p, M = self._marker_odom(m)
            m['_M'] = M
        self.mv.dk.rotate_to(h)
        return m

    def side_of(self, m, h):
        """마커(see_near 결과)가 h 방향으로 볼 때 로봇 왼쪽으로 얼마나 (m)"""
        p = self.mv.dk.pose()
        return float((m['_M'] - np.array(p[:2])) @ np.array([-math.sin(h), math.cos(h)]))

    def to_golden(self, h, tag, side_goal=0.0):
        """남쪽(h)에서 우90 -> 서쪽 벽과 수직(라이다) -> 마커 1이 기록처럼(side_goal) 보이게 통로 방향으로 옮김.
        반환 (h_w, 옮긴 거리, 마지막 마커 1)"""
        moved = 0.0
        self.turn(-90.0, f'{tag} 우90')
        h_w = self.front_square(f'{tag} 벽 수직')
        m = None
        for k in range(1, 4):
            m = self.see_near(1, h_w)
            if m is None:
                raise RuntimeError(f'{tag}: 우90 뒤 마커 1이 정면 ±20° 안에 안 보임')
            e = self.side_of(m, h_w) - side_goal            # + = 남쪽으로 가야 함
            self.log(f'[{tag} {k}] 마커 1 {m["tz"] * 100:.1f} cm, 기록과 통로 방향 {e * 100:+.1f} cm')
            if abs(e) < 0.004:
                break
            self.mv.dk.rotate_to(h)
            self.mv.move_along(e, h)
            self.mv.dk.rotate_to(h_w)
            moved += e
        return h_w, moved, m

    def wall_record(self, m, h_w):
        """마커 측정 -> 도크 좌표 기록인데 방향은 마커 기울기 대신 통로 벽(h_w = 서쪽 벽과 수직)으로
        (마커 1은 15 cm에서도 기울기가 5° 넘게 틀어져 나옴: 기록 로그 +4.7° 안 줄어듦)"""
        p, D = self._marker_odom(m)
        phi = md.wrap(h_w + math.pi)                 # 벽 법선 (벽 -> 로봇 쪽)
        rel = md.rot(-phi) @ (np.array(p[:2]) - D)
        return {'gx': float(rel[0]), 'gy': float(rel[1]), 'gth': md.wrap(p[2] - phi), 'fx': md.FX, 'size': 0.025}

    def back_to(self, tz_goal, h_w, tag):
        """서쪽 벽과 수직(h_w) 그대로 곧게 후진해 마커 1이 tz_goal인 곳 (멈춰서 재고 앞뒤만 고침)"""
        m = None
        for k in range(1, 4):
            self.mv.dk.rotate_to(h_w)
            m = self.see(1, 0.025, frames=15)
            if m is None:
                raise RuntimeError(f'{tag}: 마커 1이 안 보임')
            d = m['tz'] - tz_goal
            self.log(f'[{tag} {k}] 마커 1 {m["tz"] * 100:.1f} cm (목표 {tz_goal * 100:.1f}), 옆 {m["tx"] * 1000:+.0f} mm')
            if abs(d) < 0.003:
                break
            self.mv.move_along(d, h_w)
        self.mv.dk.rotate_to(h_w)
        return m

    def record(self):
        """D1에 손으로 (앞 = 마커 1, 바구니 = 칸막이). 로봇이:
        1. 앞 벽(라이다)과 수직 -> D1 기록 (마커 1 거리 · 옆)
        2. 곧게 앞으로 교점 쪽 (마커 1 --line-tz) -> 좌90: 마커 6이 정면 오게 앞뒤(동서) 맞춤 = 6 · 1 교점
        3. 우90 -> 벽 수직 -> 교점 기록 (마커 1)   4. 좌90 -> 벽 평행 -> 교점 기록 (마커 6, 양옆 벽)
        5. 벽 가운데로 북쪽 후진 -> 칸막이 끝 = N1 기록 (교점에서 거리). 끝: N1에서 남쪽을 봄"""
        rec = {}
        h_w = self.front_square('D1 벽 수직')
        m = self.see(1, 0.025, frames=30)
        if m is None:
            raise RuntimeError('D1에서 마커 1이 안 보임 (앞이 마커 1을 보게)')
        rec['d1_tz'], rec['d1_tx'] = m['tz'], m['tx']
        _save('~/dock_target_d1.json', self.wall_record(m, h_w))
        self.log(f'[기록] D1: 마커 1 {m["tz"] * 100:.1f} cm, 옆 {m["tx"] * 1000:+.0f} mm')
        # 2. 교점: 마커 1 줄 위에서, 남쪽을 봤을 때 마커 6이 정면
        self.back_to(self.line_tz, h_w, '교점 쪽으로 전진')
        h = None
        for k in range(1, 5):
            self.turn(90.0, f'교점 확인 {k} 좌90 (남쪽)')
            h, ws = self.wall_heading(f'교점 확인 {k}')
            m6 = self.see_near(6, h, size=0.05)
            if m6 is None:
                raise RuntimeError('남쪽을 봤는데 마커 6이 정면 ±20° 안에 안 보임')
            e = self.side_of(m6, h)                          # + = 마커 6이 동쪽 = 로봇이 서쪽에 있음 -> 동쪽(뒤)으로
            self.log(f'[교점 {k}] 마커 6 {m6["tz"] * 100:.1f} cm, 정면에서 {e * 100:+.1f} cm')
            self.turn(-90.0, f'교점 확인 {k} 우90 (서쪽)')
            h_w = self.front_square(f'교점 {k} 벽 수직')
            if abs(e) < 0.004:
                break
            self.mv.move_along(-e, h_w)
        # 3. 교점에서 마커 1
        m = self.see(1, 0.025, frames=30)
        if m is None:
            raise RuntimeError('교점에서 마커 1이 안 보임')
        rec['line_tz'], rec['line_tx'] = m['tz'], m['tx']
        rec['line_side'] = self.side_of(dict(m, _M=self._marker_odom(m)[1]), h_w)
        _save('~/dock_target_d1line.json', self.wall_record(m, h_w))
        # 4. 좌90: 마커 6 · 양옆 벽
        self.turn(90.0, '교점 좌90 (남쪽)')
        h, ws = self.wall_heading('교점 벽 방향')
        a6 = self.along6(h, '교점')
        if a6 is None:
            raise RuntimeError('교점에서 마커 6이 안 보임')
        rec['along6'] = a6
        if '오른쪽' in ws and '왼쪽' in ws:
            rec['w_west'] = (ws['오른쪽'][0] + ws['왼쪽'][0]) / 2.0
        elif '오른쪽' in ws:
            rec['w_west'] = ws['오른쪽'][0]
        # 5. 북쪽으로 후진 (가운데) -> 칸막이 끝 = N1
        _, U, end_at = self.go(END_MAX_M, '오른쪽', rec.get('w_west'), '교점 -> N1 후진', reverse=True, end_side='왼쪽')
        if end_at is None:
            raise RuntimeError(f'{U * 100:.0f} cm 후진해도 칸막이 끝이 안 나옴')
        rec['n1_dist'] = end_at
        _save('~/align_d1s.json', rec)
        self.log(f'[기록] D1 마커 1 {rec["d1_tz"] * 100:.1f} cm (옆 {rec["d1_tx"] * 1000:+.0f} mm) | 교점 마커 1 '
                 f'{rec["line_tz"] * 100:.1f} cm (옆 {rec["line_tx"] * 1000:+.0f} mm), 마커 6 {a6 * 100:.1f} cm | '
                 f'교점 -> N1(칸막이 끝) {end_at * 100:.1f} cm')
        return '기록 끝 (D1 · 교점 · N1). 로봇은 N1에서 남쪽을 봄 -> 바로 "in" 시험'

    def dock(self):
        rec = _load('~/align_d1s.json')
        h, ws = self.wall_heading('N1')
        self.go(rec['n1_dist'] + 0.30, '오른쪽', rec.get('w_west'), 'N1 -> 교점 (가운데)',
                appear_side='왼쪽', after=rec['n1_dist'])
        h = self.mv.dk.pose()[2]
        for k in range(1, 4):                         # 남북: 마커 6 거리 = 기록
            a6 = self.along6(h, f'교점 {k}')
            if a6 is None:
                break
            e = a6 - rec['along6']
            if abs(e) < 0.004:
                break
            self.mv.move_along(e, h)
            self.mv.dk.rotate_to(h)
        h, ws = self.wall_heading('교점 벽 방향')
        h_w, moved, m = self.to_golden(h, '교점', rec.get('line_side', 0.0))   # 우90, 벽 수직, 마커 1 = 기록
        m = self.back_to(rec['d1_tz'], h_w, 'D1로 후진')
        self.front_square('D1 벽 수직 확인')
        m = self.see(1, 0.025, frames=15)
        if m is not None:
            self.log(f'[D1 끝] 기록과: 앞뒤 {(m["tz"] - rec["d1_tz"]) * 1000:+.0f} mm, 옆 {(m["tx"] - rec["d1_tx"]) * 1000:+.0f} mm')
        return 'D1 도킹 끝'

    def undock(self, spin_deg=0.0):
        """D1(서쪽을 봄) -> 마커 1 거리로 곧게 앞으로 교점(통로 가운데) -> [spin_deg 있으면 우로 spin_deg + 90 이어서 돌아
        북쪽으로 끝 (AMCL 맞춤) / 없으면 우90] -> 통로 벽과 평행 북쪽 (그다음 Nav2 안내)
        v2 (빠르게, 실측 32초 -> 약 18초): 교점에서 벽 수직 다시 맞추기 · 마커 1 남북 재기(+-10 · 20° 찾기) · 회전 전 벽 평행을 뺌.
          D1에서 곧게 나오니 남북은 거의 안 바뀜 (실측 +0.4 / -0.2 cm) -> 앞으로 갈 때 잰 마커 1 옆(tx)으로 확인만,
          1 cm 넘게 틀어졌을 때만 옛 방식으로 고침. 방향은 회전 뒤 벽 평행 한 번으로 맞춤"""
        rec = _load('~/align_d1s.json')
        t0 = time.time()
        h_w = self.front_square('D1 벽 수직')
        m = self.back_to(rec['line_tz'], h_w, '교점으로 전진')
        lat = (m['tx'] - rec.get('line_tx', m['tx'])) if m is not None else 0.0
        self.log(f'[교점] 마커 1 옆 기록과 {lat * 1000:+.0f} mm')
        if abs(lat) > 0.01:                               # 드문 경우만: 옛 방식으로 남북 맞춤
            h_s = md.wrap(h_w + math.pi / 2)
            for k in range(1, 4):
                mm = self.see_near(1, h_w)
                if mm is None:
                    raise RuntimeError('교점에서 마커 1이 안 보임')
                e = self.side_of(mm, h_w) - rec.get('line_side', 0.0)
                self.log(f'[교점 {k}] 마커 1 {mm["tz"] * 100:.1f} cm, 기록과 통로 방향 {e * 100:+.1f} cm')
                if abs(e) < 0.004:
                    break
                self.mv.dk.rotate_to(h_s)
                self.mv.move_along(e, h_s)
                self.mv.dk.rotate_to(h_w)
        h_n = md.wrap(h_w - math.pi / 2)                  # 북쪽 = 서쪽에서 우90
        if spin_deg > 0:
            self.spin_here(spin_deg + 90.0, h_n, direction=-1)   # 우로 450°: 우90 + 한 바퀴를 멈추지 않고
        else:
            self.turn(-90.0, '교점 우90 (북쪽)')
        self.wall_heading('북쪽 (통로 벽)')
        self.log(f'[D1 빠지기] {time.time() - t0:.1f}초')
        return '교점에서 북쪽을 봄 (여기서 Nav2 안내)' + (f', {spin_deg:.0f}° 회전 (AMCL)' if spin_deg > 0 else '')

    def redock(self):
        """위치 맞추기(LOCALIZE) 뒤: D1 앞(서쪽을 봄)에서 앞 벽 수직 -> 마커 1 거리 = D1 기록까지 곧게 후진"""
        rec = _load('~/align_d1s.json')
        h_w = self.front_square('다시 주차 벽 수직')
        m = self.back_to(rec['d1_tz'], h_w, 'D1로 후진')
        if m is not None:
            self.log(f'[D1 다시 주차] 기록과: 앞뒤 {(m["tz"] - rec["d1_tz"]) * 1000:+.0f} mm, '
                     f'옆 {(m["tx"] - rec.get("d1_tx", m["tx"])) * 1000:+.0f} mm')
        return 'D1 다시 주차 끝'

    def spin_here(self, deg, h_end, direction=1, w=0.8):
        """제자리 회전 deg (direction +1 좌 / -1 우), 끝은 h_end. 멈추지 않고 이어서 돌다 마지막 25°만 rotate_to.
        AMCL이 벽을 여러 방향으로 보고 위치를 다시 맞춤"""
        t0 = time.time()
        p0 = self.mv.dk.pose()
        last, turned = p0[2], 0.0
        goal = math.radians(deg) - math.radians(25.0)
        while turned < goal and time.time() - t0 < 30.0:
            self.mv.dk.send(0.0, direction * w)
            time.sleep(1 / 30.0)
            y = self.mv.dk.pose()[2]
            turned += abs(md.wrap(y - last))
            last = y
        self.mv.dk.rotate_to(h_end)
        p = self.mv.dk.pose()
        self.log(f'[교점 회전] {deg:.0f}° {"우" if direction < 0 else "좌"} ({time.time() - t0:.1f}초), odom 자리 이동 '
                 f'{math.hypot(p[0] - p0[0], p[1] - p0[1]) * 1000:.0f} mm, 끝 방향 오차 '
                 f'{math.degrees(md.wrap(h_end - p[2])):+.1f}°')


def check():
    out = []
    for name in ('dock_target_d1line', 'dock_target_d1', 'align_d1s'):
        try:
            r = _load(f'~/{name}.json')
            if 'gx' in r:
                out.append(f'{name}: 앞 {r["gx"] * 100:.1f} cm, 옆 {r["gy"] * 100:+.1f} cm, '
                           f'방향 {math.degrees(md.wrap(r["gth"] - math.pi)):+.1f}° (0이면 벽과 수직)')
            else:
                out.append(f'{name}: D1 마커 1 {r.get("d1_tz", 0) * 100:.1f} cm (옆 {r.get("d1_tx", 0) * 1000:+.0f} mm), '
                           f'교점 마커 1 {r.get("line_tz", 0) * 100:.1f} cm (옆 {r.get("line_tx", 0) * 1000:+.0f} mm), '
                           f'마커 6 {r.get("along6", 0) * 100:.1f} cm, 교점 -> N1 {r.get("n1_dist", 0) * 100:.1f} cm, '
                           f'서쪽 벽(가운데) {r.get("w_west", 0) * 100:.1f} cm')
        except Exception as e:
            out.append(f'{name}: 없음 ({e})')
    return '\n'.join(out)

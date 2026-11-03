"""라이다 정답지(스캔) 맞추기: 지금 스캔을 기록한 스캔에 겹쳐서 '정답 자리에서 얼마나 벗어났나'를 잼 (2D ICP)

현업 라이다 도킹과 같은 방식: 칸 · 칸막이 · 벽 모양 전체를 한 번에 맞추니, 벽 하나 직선 맞춤보다 튼튼하고
사람 · 옆 칸 로봇처럼 기록에 없던 점은 멀어서 짝이 안 지어짐(outlier) -> 결과에 거의 안 들어감.
그래도 겹치는 점 비율(inlier)이 모자라거나 남은 오차(rms)가 크면 '못 믿음' -> 쓰는 쪽이 마커로 돌아감

정답지: records/lidar_<도크>.json  (dock_record lidar --dock d2 로 기록)
  {"target", "s_out": 도크 -> 줄 자리 거리 m, "dock": [[x, y], ...] 도크 자리 스캔, "line": [...] 줄 자리 스캔,
   "host", "date"}   점은 로봇 좌표 (x 앞, y 왼쪽, base_footprint), 반지름 SCAN_R 안만
"""
import json
import math
import os

import numpy as np

from . import records

SCAN_R = 0.8            # 이 반지름 안 점만 (멀수록 사람 · 물건이 많고 각도 오차가 커짐)
MAX_PTS = 400           # 지금 스캔: 점이 많으면 고르게 덜어냄 (라즈베리파이에서 맞추기 한 번 1초 안)
MAX_REF = 900           # 정답지 점
SCHEDULE = [0.08] * 6 + [0.04] * 8 + [0.025] * 8    # 짝 지을 최대 거리 (점점 좁힘. 라이다 잡음 ~1 cm라 2.5 cm까지만)
CELL = 0.01            # 1 cm 칸마다 점 평균 (잡음 줄임)
NORMAL_K = 12          # 벽 조각 = 정답지 점 근처 이만큼
MIN_INLIER = 0.6        # 지금 점 중 정답지 점과 2.5 cm 안에 짝이 있는 비율이 이보다 작으면 못 믿음
MAX_RMS = 0.012         # 짝들의 벽까지 남은 거리 rms (m)
MAX_T = 0.08            # 정답 자리에서 이보다 멀리 벗어났다고 나오면 못 믿음 (잘못 겹친 것)
MAX_TH = math.radians(8.0)


def trim(pts, r=SCAN_R, n=MAX_PTS, cell=CELL):
    """반지름 r 안 점 -> cell 칸마다 평균 (여러 장 합친 스캔의 잡음을 줄임) -> 많으면 고르게 n개"""
    p = np.asarray(pts, dtype=float).reshape(-1, 2)
    p = p[np.hypot(p[:, 0], p[:, 1]) < r]
    if len(p) and cell > 0:
        key = np.floor(p / cell).astype(np.int64)
        _, inv = np.unique(key, axis=0, return_inverse=True)
        inv = inv.reshape(-1)
        cnt = np.bincount(inv)
        p = np.stack([np.bincount(inv, p[:, 0]) / cnt, np.bincount(inv, p[:, 1]) / cnt], axis=1)
    if len(p) > n:
        p = p[np.linspace(0, len(p) - 1, n).astype(int)]
    return p


def compose(a, b):
    """자세 a 다음에 (a 좌표의) 자세 b"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def normals(B, k=NORMAL_K):
    """정답지 점마다 근처 k개 점으로 본 작은 벽 조각: (중심, 법선). 중심을 쓰면 정답지 점 하나하나의 잡음이 빠짐"""
    d2 = ((B[:, None, :] - B[None, :, :]) ** 2).sum(-1)
    idx = np.argsort(d2, axis=1)[:, :k]
    N = np.zeros_like(B)
    Cn = np.zeros_like(B)
    for i in range(len(B)):
        nb = B[idx[i]]
        c = nb.mean(0)
        q = nb - c
        w, v = np.linalg.eigh(q.T @ q)
        N[i] = v[:, 0]                    # 가장 작은 고윳값 방향 = 법선
        Cn[i] = c
    return Cn, N


def prep(ref):
    """정답지 준비 (점 고르기 + 벽 조각). 같은 정답지로 여러 번 맞출 때 한 번만"""
    B = trim(ref, n=MAX_REF)
    if len(B) < 30:
        return B, B, B
    BC, NB = normals(B)
    return B, BC, NB


def icp(cur, ref, th0=0.0, max_th=MAX_TH, max_t=MAX_T):
    """지금 스캔 cur를 정답지 ref에 겹침 (점-선 ICP: 벽을 따라 미끄러지는 오차가 없음).
    반환 dict(x, y, th = 정답 자리 기준 지금 로봇 자세 (x 앞, y 왼쪽), inlier, rms, ok, why). 점은 로봇 좌표 (N, 2)"""
    A0 = trim(cur)
    B, BC, NB = ref if isinstance(ref, tuple) else prep(ref)
    if len(A0) < 30 or len(B) < 30:
        return {'ok': False, 'why': f'점이 모자람 (지금 {len(A0)}, 정답 {len(B)})'}
    x = np.array([0.0, 0.0, th0])         # tx, ty, th : 지금 점 -> 정답 좌표 (th0 = 처음 짐작한 각)
    m = d = None
    for d_max in SCHEDULE:
        c, s_ = math.cos(x[2]), math.sin(x[2])
        P = A0 @ np.array([[c, -s_], [s_, c]]).T + x[:2]
        d2 = ((P[:, None, :] - B[None, :, :]) ** 2).sum(-1)
        j = np.argmin(d2, axis=1)
        d = np.sqrt(d2[np.arange(len(P)), j])
        m = d < d_max
        if m.sum() < 20:
            return {'ok': False, 'why': f'겹치는 점이 모자람 ({int(m.sum())})'}
        p, q, n = P[m], BC[j[m]], NB[j[m]]
        r = ((p - q) * n).sum(1)
        J = np.stack([n[:, 0], n[:, 1], n[:, 1] * p[:, 0] - n[:, 0] * p[:, 1]], axis=1)
        dx = np.linalg.lstsq(J.T @ J + 1e-9 * np.eye(3), -J.T @ r, rcond=None)[0]
        # 작은 회전 dth를 지금 변환 앞에 붙임: P' = Rd P + td
        cd, sd = math.cos(dx[2]), math.sin(dx[2])
        Rd = np.array([[cd, -sd], [sd, cd]])
        x[:2] = Rd @ x[:2] + dx[:2]
        x[2] = wrap(x[2] + dx[2])
    inl = float(m.mean())
    rms = float(np.sqrt(np.mean(r ** 2)))
    out = {'x': float(x[0]), 'y': float(x[1]), 'th': float(x[2]), 'inlier': inl, 'rms': rms, 'n': len(A0)}
    why = []
    if inl < MIN_INLIER:
        why.append(f'겹침 {inl * 100:.0f}% (사람 · 옆 칸 로봇 · 물건?)')
    if rms > MAX_RMS:
        why.append(f'오차 {rms * 1000:.1f} mm')
    if math.hypot(x[0], x[1]) > max_t or abs(x[2]) > max_th:
        why.append(f'너무 멀리 벗어남 ({x[0] * 100:+.1f}, {x[1] * 100:+.1f} cm, {math.degrees(x[2]):+.1f}°)')
    out['ok'] = not why
    out['why'] = ', '.join(why)
    return out


def icp_search(cur, ref, search_deg=45.0, step_deg=5.0, max_t=MAX_T):
    """처음 각을 모를 때 (제자리 회전 시험): -search ~ +search를 step마다 처음 각으로 넣어 보고 가장 잘 겹친 것"""
    best = None
    ref = ref if isinstance(ref, tuple) else prep(ref)
    for d in np.arange(-search_deg, search_deg + 1e-6, step_deg):
        r = icp(cur, ref, th0=math.radians(d), max_th=math.radians(search_deg + 10), max_t=max_t)
        if 'rms' not in r:
            continue
        key = (r['ok'], r['inlier'], -r['rms'])
        if best is None or key > best[0]:
            best = (key, r)
    return best[1] if best else {'ok': False, 'why': '겹치기 실패'}


def load(target):
    """정답지 (없으면 None)"""
    if not target:
        return None
    path = records.find(f'lidar_{target}.json')
    if not os.path.exists(path):
        return None
    with open(path) as f:
        g = json.load(f)
    return g if g.get('line') and g.get('dock') else None


def collect(lidar, secs, sleep):
    """멈춘 채 secs초 동안 스캔 모으기 (여러 장 합쳐서 점을 촘촘히)"""
    import time
    acc, t_end = [], time.time() + secs
    while time.time() < t_end:
        q = lidar.take() if lidar is not None else None
        if q is not None:
            acc.append(np.asarray(q))
        sleep(0.03)
    return np.concatenate(acc) if acc else None

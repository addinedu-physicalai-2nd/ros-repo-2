"""D2 · D3 후면 주차 (11/3: 칸 벽에 살짝 닿아 안 들어감) 시험, ROS 없이:  python3 -m pytest test/test_pocket_park.py

가짜 세상 (odom = 세상): 마커는 원점 (통로 건너 칸막이 x = 0), 벽 -> 칸 방향 = +x (phi = 0).
로봇은 마커를 보고(-x) +x로 후진해 칸(x 0.27 ~ 0.50, 벽 y = ±0.11)에 들어감. 정답 도크 = (0.36, 0, π)
카메라는 멀수록 옆으로 밀려 보이게 (2° 치우침) = 11/3 로그처럼 후진할수록 '옆'이 커짐"""
import math
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet_agent.docking import corridor as C  # noqa: E402
from fleet_agent.docking import marker_dock as md  # noqa: E402
from fleet_agent.docking import scan_match as sm  # noqa: E402

BIAS = math.radians(2.0)
DOCK = (0.36, 0.0, math.pi)
S_OUT = 0.175


def seg(a, b, step=0.005):
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return np.stack([np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)], axis=1)


WALLS = np.concatenate([
    seg((0.0, -0.45), (0.0, 0.45)),          # 통로 건너 칸막이 (마커)
    seg((0.27, 0.11), (0.50, 0.11)),         # 칸 왼쪽 벽 (세상 +y)
    seg((0.27, -0.11), (0.50, -0.11)),       # 칸 오른쪽 벽
    seg((0.50, -0.11), (0.50, 0.11)),        # 칸 안쪽 벽
    seg((0.27, 0.11), (0.27, 0.45)),         # 옆 칸과 사이 칸막이 앞면
    seg((0.27, -0.45), (0.27, -0.11)),
])


class World:
    def __init__(self, x, y, th):
        self.p = [x, y, th]
        self.extra = None                     # 세상에 더 있는 점 (사람 · 옆 칸 로봇)

    def step(self, v, w, dt=1 / 15):
        self.p[2] = md.wrap(self.p[2] + w * dt)
        self.p[0] += v * math.cos(self.p[2]) * dt
        self.p[1] += v * math.sin(self.p[2]) * dt


class Dk:
    def __init__(self, w):
        self.w = w
        self.cam = None

    def pose(self):
        return tuple(self.w.p)

    def rotate_to(self, th):
        self.w.p[2] = th
        return 0.0

    def find_marker(self):
        return True


class Mv:
    def __init__(self, w):
        self.w = w
        self.dk = Dk(w)
        self.ir = {}

    def send(self, v, w):
        self.w.step(v, w)

    def stop(self, s=0.0):
        pass

    def move_along(self, d, h):
        self.w.p[0] += d * math.cos(h)
        self.w.p[1] += d * math.sin(h)

    def drive(self, dist, reverse, h, vmax, **k):
        self.move_along(-dist if reverse else dist, h)
        return 'dist', dist, None


class Cam:
    def __init__(self, w, bias=BIAS):
        self.w, self.bias, self.id = w, bias, 2

    def measure(self, raw=False):
        x, y, th = self.w.p
        c, s = math.cos(-th), math.sin(-th)
        fx, fy = c * -x - s * -y, s * -x + c * -y
        tz = fx - md.CAM_X_M
        if tz < 0.03:
            return None
        return {'tz': tz, 'tx': -fy + tz * math.tan(self.bias), 'yaw': 0.0}

    def measure_stable(self, frames=10, **k):
        return self.measure()


class Lidar:
    """세상 벽 점을 로봇 좌표로 + 잡음 (RPLIDAR 근거리 ~1 cm 수준)"""
    NOISE = 0.008

    def __init__(self, w, seed=0):
        self.w = w
        self.rng = np.random.default_rng(seed)

    def take(self):
        x, y, th = self.w.p
        P = WALLS if self.w.extra is None else np.concatenate([WALLS, self.w.extra])
        d = P - np.array([x, y])
        c, s = math.cos(-th), math.sin(-th)
        q = np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], axis=1)
        body = (np.abs(q[:, 0]) < 0.14) & (np.abs(q[:, 1]) < 0.09)
        q = q[~body]
        q = q + self.rng.normal(0.0, self.NOISE, q.shape)
        return q[np.hypot(q[:, 0], q[:, 1]) < 1.0]


STOP = {'gx': 0.185, 'gy': 0.0, 'gth': math.pi, 'size': 0.025}
TARGET = {'gx': 0.36, 'gy': 0.0, 'gth': math.pi, 'size': 0.025}


@pytest.fixture
def fast(monkeypatch, tmp_path):
    monkeypatch.setattr(time, 'sleep', lambda s: None)
    t = [1000.0]

    def fake_time():
        t[0] += 0.02
        return t[0]
    monkeypatch.setattr(time, 'time', fake_time)
    monkeypatch.setattr(C, '_set_size', lambda s: None)
    monkeypatch.setenv('HOME', str(tmp_path))
    return tmp_path


def make_gold():
    """정답 도크에 손으로 넣고 dock_record lidar 한 것과 같게"""
    w = World(*DOCK)
    st = C.Steps(Mv(w), None, lambda s: None, lidar=Lidar(w))
    saved = {}
    orig = C._save
    C._save = lambda path, d: saved.update(d=d)
    try:
        msg = st.record_pocket('d2', S_OUT, secs=0.3)
    finally:
        C._save = orig
    assert w.p[0] == pytest.approx(DOCK[0] - S_OUT)          # 줄 자리로 나옴
    assert '저장' in msg
    return saved['d']


def run(monkeypatch, gold=None, start=(0.185, 0.0, math.pi), extra=None, trust=None, stop=STOP, target=TARGET):
    monkeypatch.setattr(C.sm, 'load', lambda t: gold)
    if trust is not None:
        monkeypatch.setattr(C, 'PARK_TRUST_Z', trust)
    w = World(*start)
    w.extra = extra
    logs = []
    st = C.Steps(Mv(w), Cam(w), logs.append, lidar=Lidar(w))
    st.park(2, stop, target, 'N16', phi=0.0, target='d2')
    return w, logs


# ---------------------------------------------------------------- 스캔 맞추기
def scans(w, k=8, seed=0):
    li = Lidar(w, seed)
    return np.concatenate([li.take() for _ in range(k)])


def test_icp_finds_offset():
    w = World(0.185 - 0.005, -0.006, math.pi + math.radians(1.5))
    ref = sm.trim(scans(World(0.185, 0.0, math.pi), 40, 1), n=sm.MAX_REF)
    r = sm.icp(scans(w), ref)
    assert r['ok']
    # 정답 자리(앞 = -x 세상) 기준: 세상 -x로 5 mm = 앞 +5 mm, 세상 -y 6 mm = 로봇 왼쪽(+y 로봇)? 로봇이 -x를 보면 왼쪽 = 세상 -y
    assert r['x'] == pytest.approx(0.005, abs=0.003)
    assert r['y'] == pytest.approx(0.006, abs=0.003)
    assert math.degrees(r['th']) == pytest.approx(1.5, abs=0.4)


def test_icp_survives_neighbor_robot_but_rejects_mess():
    ref = sm.trim(scans(World(0.185, 0.0, math.pi), 40, 1), n=sm.MAX_REF)
    w = World(0.185, 0.003, math.pi)
    w.extra = np.concatenate([seg((0.30, 0.18), (0.45, 0.18)), seg((0.30, 0.30), (0.45, 0.30))])   # 옆 칸 로봇
    r = sm.icp(scans(w), ref)
    assert r['ok'] and r['y'] == pytest.approx(-0.003, abs=0.003)
    rng = np.random.default_rng(0)
    junk = rng.uniform(-0.6, 0.6, (900, 2))                                                   # 사람 · 물건 잔뜩
    r2 = sm.icp(np.concatenate([scans(World(0.185, 0.0, math.pi)), junk]), ref)
    assert not r2['ok'] and '겹침' in r2['why']


# ---------------------------------------------------------------- 주차
def test_marker_only_far_frames_not_trusted(monkeypatch, fast):
    old, _ = run(monkeypatch, trust=9.0)
    new, _ = run(monkeypatch)
    assert abs(new.p[1]) <= abs(old.p[1]) + 1e-4
    assert new.p[0] == pytest.approx(DOCK[0], abs=0.005)


def test_lidar_gold_parks_on_pocket_center(monkeypatch, fast):
    """줄 자리에서 옆으로 6 mm 틀어져 있고 카메라도 치우쳐 있어도 -> 칸 가운데 (마커만이면 못 고침)"""
    gold = make_gold()
    start = (0.185, 0.006, math.pi + math.radians(1.0))
    mk, _ = run(monkeypatch, gold=None, start=start)
    li, logs = run(monkeypatch, gold=gold, start=start)
    assert abs(li.p[1]) < 0.003 and abs(li.p[1]) < abs(mk.p[1])
    assert li.p[0] == pytest.approx(DOCK[0], abs=0.004)
    assert math.degrees(md.wrap(li.p[2] - math.pi)) == pytest.approx(0.0, abs=0.5)
    assert any('줄 자리 옮김' in s for s in logs) and any('라이다 끝' in s for s in logs)
    csv_txt = open(os.path.join(fast, 'dock_logs', 'pocket_lidar.csv'), encoding='utf-8').read()
    assert '라이다' in csv_txt and 'd2' in csv_txt


def test_lidar_falls_back_to_marker_when_messy(monkeypatch, fast):
    gold = make_gold()
    rng = np.random.default_rng(1)
    junk = rng.uniform(-0.6, 0.8, (1500, 2))
    w, logs = run(monkeypatch, gold=gold, extra=junk)
    assert any('못 믿음' in s for s in logs)
    assert w.p[0] == pytest.approx(DOCK[0], abs=0.006)        # 마커로 그대로 주차됨


def test_reverse_line_follows_pocket_axis(monkeypatch, fast):
    gth = math.pi - math.radians(1.5)
    w, _ = run(monkeypatch, stop=dict(STOP, gth=gth), target=dict(TARGET, gth=gth))
    assert math.degrees(md.wrap(w.p[2] - gth)) == pytest.approx(0.0, abs=0.3)


def test_no_gold_still_reports(monkeypatch, fast):
    w, logs = run(monkeypatch, gold=None)
    assert any('정답지 없음' in s for s in logs)

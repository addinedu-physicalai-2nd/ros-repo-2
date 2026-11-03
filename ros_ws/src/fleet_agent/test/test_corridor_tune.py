"""11/3 로그 기반 튜닝 시험, ROS 없이:  python3 -m pytest test/test_corridor_tune.py

  - P2 -> P1: 예상 선 10 cm 앞까지는 8 cm/s (예전: 내내 4 cm/s)
  - 출발 전 벽 방향: 벽이 안 보이면 0.5초만 보고 출발, 5° 넘는 고침은 무시
  - N14 · D1 빠지기: 반복 측정 줄임 (허용 오차 · odom 믿기)
  - 주차 실패 -> 시작 자리로 돌아가 한 번 더, 취소는 다시 안 함
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet_agent.docking import corridor as C  # noqa: E402
from fleet_agent.docking import marker_dock as md  # noqa: E402
from fleet_agent.docking.entrance import Entrance  # noqa: E402


# ---------------------------------------------------------------- 가짜 로봇
class FakeDk:
    def __init__(self):
        self.p = [0.0, 0.0, 0.0]
        self.rotations = []

    def pose(self):
        return tuple(self.p)

    def rotate_to(self, th):
        self.rotations.append(th)
        self.p[2] = th
        return 0.0


class FakeMv:
    def __init__(self):
        self.dk = FakeDk()
        self.drives = []
        self.moves = []
        self.ir = {}

    def stop(self, settle=0.0):
        pass

    def drive(self, dist, reverse, heading, vmax, **k):
        self.drives.append((dist, reverse))
        return 'dist', dist, None

    def move_along(self, d, h):
        self.moves.append(d)


class FakeCam:
    """measure_stable이 부를 때마다 차례로 값을 돌려줌"""

    def __init__(self, seq):
        self.seq = list(seq)
        self.n = 0
        self.id = None

    def measure_stable(self, frames=8, **k):
        self.n += 1
        return self.seq.pop(0) if self.seq else None


def mk(tz=0.29, yaw=0.0, tx=0.0):
    return {'tz': tz, 'yaw': yaw, 'tx': tx}


def steps(cam_seq, cls=C.Steps):
    mv = FakeMv()
    cam = FakeCam(cam_seq)
    mv.dk.cam = None
    st = cls(mv, cam, lambda s: None, lidar=None)
    return st, mv, cam


# ---------------------------------------------------------------- 1 · 2. 통로 속도 · 가로선 거리
def test_bar_gap_matches_log():
    assert C.BAR_GAP['N15'] == pytest.approx(0.36)
    assert C.BAR_GAP['P1'] == pytest.approx(0.37)


def test_p1_fast_until_near():
    assert C.bar_vmax('P1', 'P1', 0.30) == C.FOLLOW_V        # 예전엔 여기서도 4 cm/s
    assert C.bar_vmax('P1', 'P1', 0.08) == C.V_BAR_SLOW
    assert C.bar_vmax('P1', 'P1', -0.02) == C.V_BAR_SLOW     # 선을 지나쳐도 천천히


def test_other_nodes_unchanged():
    assert C.bar_vmax('N15', 'N16', 0.02) == C.FOLLOW_V       # 지나갈 선은 빠르게
    assert C.bar_vmax('N16', 'N16', 0.05) == C.V_BAR_SLOW     # 설 선 6 cm 앞
    assert C.bar_vmax('N16', 'N16', 0.10) == C.FOLLOW_V
    assert C.bar_vmax('P2', 'P2', 0.0) == pytest.approx(0.02)  # 거리로 서는 노드는 줄어듦


# ---------------------------------------------------------------- 3. 반복 측정 줄이기
def test_n14_align_fewer_measurements(monkeypatch):
    monkeypatch.setattr(C, '_set_size', lambda s: None)
    front = {'id': 4, 'size': 0.05, 'tz': 0.293, 'yaw': 0.0, 'bearing': 0.0}
    axis = {'id': 5, 'size': 0.05, 'tz': 0.16, 'yaw': 0.0, 'bearing': 0.0}
    # 마커 4: -2.2° (한 번 돌면 끝) -> 거리 27.9 cm (1.4 cm 뒤로, odom 믿음) -> 마커 5: 0.2° (안 돎)
    st, mv, cam = steps([mk(0.279, math.radians(-2.2)), mk(0.279), mk(0.16, math.radians(0.2))])
    st.n14_align(front, axis)
    assert cam.n == 3                    # 마커 4 한 번 + 거리 한 번 + 마커 5 한 번
    assert len(mv.drives) == 1


def test_n14_reach_still_fixes_big_error(monkeypatch):
    monkeypatch.setattr(C, '_set_size', lambda s: None)
    front = {'id': 4, 'size': 0.05, 'tz': 0.293, 'yaw': 0.0, 'bearing': 0.0}
    st, mv, cam = steps([mk(0.24), mk(0.29)])             # 5.3 cm 차이 -> 움직이고 다시 잼
    st.reach(front, 't', tol_m=0.01, trust_m=0.03)
    assert cam.n == 2 and len(mv.drives) == 1


def test_d1_exit_back_to_one_move(monkeypatch):
    monkeypatch.setattr(C, '_set_size', lambda s: None)
    st, mv, cam = steps([mk(0.277), mk(0.131), mk(0.128)], cls=Entrance)
    st.back_to(0.127, 0.0, 't', tol_m=0.01, trust_m=0.20)
    assert cam.n == 1 and len(mv.moves) == 1             # 예전: 3번 재고 2번 움직임
    st2, mv2, cam2 = steps([mk(0.277), mk(0.131), mk(0.128)], cls=Entrance)
    st2.back_to(0.127, 0.0, 't')                          # 기본값(기록 도구)은 예전 그대로
    assert cam2.n == 3


# ---------------------------------------------------------------- 출발 전 벽 방향
class FakeLidar:
    def __init__(self, pts):
        self.pts = pts

    def take(self):
        return self.pts


def _start_follow(monkeypatch, lidar, wall_ang_deg):
    st, mv, cam = steps([])
    st.lidar = lidar
    mv.ir = {'on': False, 'bar_n': 0}
    angs = {'왼쪽': (0.17, math.radians(wall_ang_deg), 30, 0.001)}
    monkeypatch.setattr(C, 'side_lines', lambda pts, along=0.3: angs)
    import time as _t
    stamps = []

    class Stop(Exception):
        pass

    def send(v, w):
        stamps.append(_t.time())
        raise Stop
    mv.send = send
    t0 = _t.time()
    with pytest.raises(Stop):
        st.follow_bars(lambda: ['N15'], 'N14')
    return mv, stamps[0] - t0


def test_prestart_no_wall_waits_short(monkeypatch):
    mv, dt = _start_follow(monkeypatch, FakeLidar(None), 0.0)
    assert dt < 0.9                      # 예전 1.5초
    assert mv.dk.rotations == []


def test_prestart_rejects_big_correction(monkeypatch):
    mv, _ = _start_follow(monkeypatch, FakeLidar(np.zeros((1, 2))), 14.6)
    assert mv.dk.rotations == []         # 14.6°는 잘못 본 것 -> 안 돎


def test_prestart_small_correction_applied(monkeypatch):
    mv, _ = _start_follow(monkeypatch, FakeLidar(np.zeros((1, 2))), -1.6)
    assert mv.dk.rotations and math.degrees(mv.dk.rotations[0]) == pytest.approx(-1.6, abs=0.01)


# ---------------------------------------------------------------- 9. 주차 다시 하기
def test_park_retry_returns_to_start(monkeypatch):
    st, mv, cam = steps([])
    mv.dk.p = [1.0, 2.0, 0.5]
    calls = []
    gone = []

    def once(*a):
        calls.append(1)
        mv.dk.p = [0.9, 2.0, 0.5]         # 칸 안으로 조금 들어간 뒤
        if len(calls) == 1:
            raise RuntimeError('주차 중 마커를 1초 넘게 못 봄')
    monkeypatch.setattr(st, '_park_once', once)
    monkeypatch.setattr(st, 'go_to', lambda x, y, vmax=0.05: gone.append((x, y)))
    st.park(2, {}, {}, 'N16')
    assert len(calls) == 2
    assert gone == [(1.0, 2.0)]          # 시작 자리(줄 위)로 돌아가서 다시


def test_park_retry_gives_up_after_one(monkeypatch):
    st, mv, cam = steps([])
    monkeypatch.setattr(st, '_park_once', lambda *a: (_ for _ in ()).throw(RuntimeError('x')))
    monkeypatch.setattr(st, 'go_to', lambda x, y, vmax=0.05: None)
    with pytest.raises(RuntimeError):
        st.park(2, {}, {}, 'N16')


def test_park_cancel_not_retried(monkeypatch):
    st, mv, cam = steps([])
    calls = []

    def once(*a):
        calls.append(1)
        raise md.Cancelled()
    monkeypatch.setattr(st, '_park_once', once)
    with pytest.raises(md.Cancelled):
        st.park(2, {}, {}, 'N16')
    assert len(calls) == 1

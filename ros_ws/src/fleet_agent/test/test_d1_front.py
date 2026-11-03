"""D1 교점: 서쪽으로 돈 뒤 라이다 앞 벽 거리로 동서를 먼저 맞춤, ROS 없이:  python3 -m pytest test/test_d1_front.py"""
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet_agent.docking import entrance as E  # noqa: E402


class Dk:
    def pose(self):
        return (0.0, 0.0, 0.0)

    def rotate_to(self, th):
        return 0.0


class Mv:
    def __init__(self):
        self.dk = Dk()
        self.dk.cam = None
        self.moves = []

    def move_along(self, d, h):
        self.moves.append(d)


class Lidar:
    def __init__(self, d):
        self.d = d

    def take(self):
        if self.d is None:
            return None
        y = np.linspace(-0.14, 0.14, 40)
        return np.stack([np.full_like(y, self.d), y], axis=1)


def make(d):
    mv = Mv()
    logs = []
    return E.Entrance(mv, None, logs.append, lidar=Lidar(d)), mv, logs


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    t = [0.0]

    def ft():
        t[0] += 0.05
        return t[0]
    monkeypatch.setattr(time, 'time', ft)
    monkeypatch.setattr(time, 'sleep', lambda s: None)


def test_moves_forward_when_far():          # 11/3 pinky1: 서쪽 벽 25 cm, 줄 자리 16.2 cm
    st, mv, logs = make(0.250)
    st.front_reach(0.162, 0.0, '교점 동서')
    assert mv.moves == [pytest.approx(0.088, abs=1e-3)]


def test_small_error_left_to_marker():
    st, mv, logs = make(0.168)
    st.front_reach(0.162, 0.0, '교점 동서')
    assert mv.moves == []


def test_no_wall_or_crazy_value_does_nothing():
    st, mv, logs = make(None)
    st.front_reach(0.162, 0.0, '교점 동서')
    st2, mv2, _ = make(0.45)                # 15 cm 넘게 틀리면 잘못 본 것
    st2.front_reach(0.162, 0.0, '교점 동서')
    assert mv.moves == [] and mv2.moves == []

"""제자리 회전 시험 (odom 회전 vs 라이다로 본 실제 회전), ROS 없이:  python3 -m pytest test/test_spin_check.py"""
import math
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))

from fleet_agent.docking import spin_check  # noqa: E402
from test_pocket_park import WALLS  # noqa: E402


class Robot:
    """실제 회전 = 명령 회전, odom은 k배로 부풀려 잼 (바퀴 간격 설정이 실제보다 작을 때)"""

    def __init__(self, k_odom, skew=0.0):
        self.true = [0.20, 0.0, math.pi]
        self.odom_th = math.pi
        self.k, self.skew = k_odom, skew
        self.dk = self

    def pose(self):
        return (self.true[0], self.true[1], self.odom_th)

    def send(self, v, w, dt=1 / 30.0):
        self.true[2] += w * dt
        self.odom_th = (self.odom_th + w * dt * self.k * (1 + self.skew * np.sign(w)) + math.pi) % (2 * math.pi) - math.pi

    def stop(self, s=0.0):
        pass


class Lidar:
    def __init__(self, r, noise=0.006):
        self.r = r
        self.rng = np.random.default_rng(0)
        self.noise = noise

    def take(self):
        x, y, th = self.r.true
        d = WALLS - np.array([x, y])
        c, s = math.cos(-th), math.sin(-th)
        q = np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], axis=1)
        q = q[~((np.abs(q[:, 0]) < 0.14) & (np.abs(q[:, 1]) < 0.09))]
        return q + self.rng.normal(0, self.noise, q.shape)


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    t = [0.0]

    def ft():
        t[0] += 1 / 30.0
        return t[0]
    monkeypatch.setattr(time, 'time', ft)
    monkeypatch.setattr(time, 'sleep', lambda s: None)


@pytest.mark.parametrize('k', [1.03, 0.98, 1.0])
def test_ratio_found(k):
    r = Robot(k)
    out = spin_check.run(r, Lidar(r), lambda s: None, turns=1, w=0.6, secs=0.5)
    # 실제/odom = 1/k
    assert out['ratio'] == pytest.approx(1 / k, abs=0.002)
    assert '바퀴 간격' in out['msg']


def test_two_turns_big_error():
    r = Robot(1.06)                       # 90°에 약 5° 틀리는 정도 (11/3 pinky2 N14 +6.7°)
    out = spin_check.run(r, Lidar(r), lambda s: None, turns=2, w=0.6, secs=0.5)
    assert out['ratio'] == pytest.approx(1 / 1.06, abs=0.002)

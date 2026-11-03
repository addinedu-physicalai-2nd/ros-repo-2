"""D1 그림자 측정 (마커 6 옆에 라이다 앞 벽 거리 기록, 동작은 안 바뀜), ROS 없이:  python3 -m pytest test/test_d1_shadow.py"""
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet_agent.docking import entrance as E  # noqa: E402


class FakeDk:
    def pose(self):
        return (0.0, 0.0, 0.0)


class FakeMv:
    def __init__(self):
        self.dk = FakeDk()
        self.moved = False

    def send(self, v, w):
        self.moved = True


class FakeLidar:
    def __init__(self, pts):
        self.pts = pts

    def take(self):
        return self.pts


def wall(d):
    """로봇 앞 d m에 벽 (옆 ±14 cm 점 40개)"""
    y = np.linspace(-0.14, 0.14, 40)
    return np.stack([np.full_like(y, d), y], axis=1)


def make(pts):
    mv = FakeMv()
    mv.dk.cam = None
    logs = []
    st = E.Entrance(mv, None, logs.append, lidar=FakeLidar(pts))
    return st, mv, logs


def rows(home):
    with open(os.path.join(home, 'dock_logs', 'shadow_d1.csv'), encoding='utf-8') as f:
        return list(csv.DictReader(f))


def test_shadow_logs_marker_and_lidar(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setattr(E, 'SHADOW_SECS', 0.15)
    st, mv, logs = make(wall(0.30))
    st.shadow_front('교점 1', 0.318, 0.300)
    st.shadow_front('교점 2', 0.303, 0.300)
    r = rows(str(tmp_path))
    assert len(r) == 2
    assert r[0]['라이다_cm'] == '30.00' and r[0]['마커6_cm'] == '31.80' and r[0]['마커-라이다_cm'] == '1.80'
    assert any('그림자' in s and '30.0 cm' in s for s in logs)
    assert not mv.moved                                   # 로봇은 안 움직임


def test_shadow_no_wall_still_logs(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setattr(E, 'SHADOW_SECS', 0.1)
    st, mv, logs = make(None)
    st.shadow_front('교점 1', 0.31, 0.30)
    r = rows(str(tmp_path))
    assert r[0]['라이다_cm'] == '' and r[0]['마커6_cm'] == '31.00'
    assert any('안 잡힘' in s for s in logs)


def test_shadow_never_raises(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    st, mv, logs = make(wall(0.3))

    class Bad:
        def take(self):
            raise RuntimeError('scan 끊김')
    st.lidar = Bad()
    st.shadow_front('교점 1', None, 0.30)                  # 도킹을 멈추게 하면 안 됨
    assert any('기록 실패' in s for s in logs)

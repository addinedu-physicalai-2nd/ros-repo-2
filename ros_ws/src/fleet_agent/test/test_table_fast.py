"""테이블 정지선: 앞 선을 지난 뒤 fast_gap까지는 8 cm/s, 그다음 4 cm/s, ROS 없이:  python3 -m pytest test/test_table_fast.py"""
import math
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet_agent.docking import table_dock as T  # noqa: E402

LINES = (0.04, 0.29)          # 11/3 로그처럼 선 사이 25 cm
TAPE = 0.02


class Dk:
    def __init__(self, ir):
        self.p = [0.0, 0.0, 0.0]
        self.ir = ir
        self.vs = []

    def pose(self):
        return tuple(self.p)

    def send(self, v, w):
        self.vs.append((self.p[0], v))
        self.p[0] += v / 30.0
        on = any(a <= self.p[0] <= a + TAPE for a in LINES)
        self.ir['v'] = [4095] * 3 if on else [240] * 3
        self.ir['n'] += 1

    def stop(self):
        pass


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    t = [0.0]

    def ft():
        t[0] += 1 / 30.0
        return t[0]
    monkeypatch.setattr(time, 'time', ft)
    monkeypatch.setattr(time, 'sleep', lambda s: None)
    monkeypatch.setattr(T.md, 'log', lambda s: None)


def drive(use_fast):
    ir = {'v': [240] * 3, 'n': 0}
    c = dict(T.DEFAULTS)
    dk = Dk(ir)
    lc = T.LineCounter(ir, c)
    lc.start(dk.pose())
    td = T.TableDocker.__new__(T.TableDocker)
    td.c = c
    t0 = time.time()
    hit = td._drive(dk, lc, 0.40, 0.0, c['v_line'], stop_on=2, c=c,
                    fast=(c['v_line_fast'], c['fast_gap']) if use_fast else None)
    return hit, dk, time.time() - t0


def test_faster_and_same_stop():
    hit0, dk0, t_slow = drive(False)
    hit1, dk1, t_fast = drive(True)
    assert hit0 and hit1
    assert dk1.p[0] == pytest.approx(dk0.p[0], abs=0.004)          # 같은 선에서 섬
    assert t_fast < t_slow - 1.5                                    # 1.5초 넘게 빨라짐
    v_at_line2 = [v for x, v in dk1.vs if x > LINES[1] - 0.02]
    assert max(v_at_line2) <= T.DEFAULTS['v_line'] + 1e-9          # 설 선 근처는 천천히
    assert max(v for x, v in dk1.vs) == pytest.approx(T.DEFAULTS['v_line_fast'], abs=0.005)

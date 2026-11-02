"""장애물 대기 · 우회 (로봇 쪽):  python3 -m pytest test/test_obstacle.py -v

- 막히면 사람 · 물건 상관없이 block_wait_sec(10초) 기다림, 3초부터 LCD '지나갈게요'
- 그 전에 비키면 서버에 알리지 않고 그대로 감
- 10초 지나도 있으면 BLOCKED(막힌 노드) 한 번 -> 서버가 [지금 노드, 새 길...]을 REPLACE로 주면
  지금 노드로 돌아갔다가 새 길로 간다 (서버 쪽: TrafficManager::reroute, main_service_sim SIM_BLOCK)
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))

from fleet_agent.core import (CMD_DONE, EV_BLOCKED, FORWARD, NODE_REACHED, BlockPolicy, Driver,  # noqa: E402
                              ReverseControl)
from test_core import FakeRobot, run  # noqa: E402

N1, N2, N3, N4 = ('N1', 0.0, 0.0, 0.0), ('N2', 1.0, 0.0, 0.0), ('N3', 0.0, 1.0, 1.5708), ('N4', 1.0, 1.0, 0.0)


def make(x=0.0, y=0.0, yaw=0.0):
    r = FakeRobot(x, y, yaw)
    d = Driver(r, {}, reverse=ReverseControl(speed=0.1), policy=BlockPolicy(wait=10.0))
    d.last_drive = FORWARD
    d.last_node = 'N1'
    r.driver = d
    return r, d


def reached(r):
    return [n for t, n, _ in r.events if t == NODE_REACHED]


def test_policy_waits_10s_for_any_kind_and_asks_from_3s():
    for kind in ('STATIC', 'DYNAMIC', 'UNKNOWN'):
        p = BlockPolicy(wait=10.0)
        asks, reports = [], []
        for i in range(150):
            a, rep = p.update(i * 0.1, kind)
            asks.append(a)
            if rep:
                reports.append(round(i * 0.1, 1))
        assert reports == [10.0], kind
        assert not asks[20] and asks[35] and asks[95] and not asks[110], kind


def test_policy_kind_change_keeps_same_clock():
    # LiDAR는 처음 UNKNOWN -> 2초 뒤 STATIC: 시계는 처음부터 (합쳐 10초)
    p = BlockPolicy(wait=10.0)
    reports = [i * 0.1 for i in range(150) if p.update(i * 0.1, 'UNKNOWN' if i < 20 else 'STATIC')[1]]
    assert len(reports) == 1 and abs(reports[0] - 10.0) < 0.15


def test_old_policy_still_works_when_wait_is_zero():
    p = BlockPolicy(dynamic_wait=3, dynamic_ask=5, static_wait=2, wait=0.0)
    assert [i * 0.1 for i in range(50) if p.update(i * 0.1, 'STATIC')[1]][0] < 2.15


def test_obstacle_moves_away_before_10s_no_report():
    r, d = make()
    d.on_move(1, False, FORWARD, [N2], False, 'N2')

    def hook(t):
        d.set_obstacle('STATIC' if 1.0 <= t < 6.0 else '')
    run(r, d, 20, hook=hook)
    assert [e for e in r.events if e[0] == EV_BLOCKED] == []
    assert reached(r) == ['N2'] and any(r.lcd), '3초부터 LCD 지나갈게요'


def test_blocked_10s_then_server_detour_goes_back_and_around():
    r, d = make()
    d.on_move(1, False, FORWARD, [N2, N4], False, 'N4')
    seen = {}

    def hook(t):
        blocked = [e for e in r.events if e[0] == EV_BLOCKED]
        if not blocked:
            d.set_obstacle('STATIC' if 1.0 <= t else '')        # N2로 가다 막힘 (안 비킴)
            return
        if 'replace' not in seen:                                # 서버 reroute: [N1(지금 노드), N3, N4]
            seen['replace'] = t
            seen['blocked'] = blocked[0]
            seen['x'] = r.x
            d.on_move(2, False, FORWARD, [N1], False, 'N4')
            d.on_move(2, True, FORWARD, [N3, N4], False, 'N4')
        d.set_obstacle('')                                       # 돌아서면 장애물은 뒤 (앞 부채꼴 밖)
    run(r, d, 60, hook=hook)
    assert seen['blocked'][1] == 'N2' and seen['blocked'][2].startswith('STATIC:N1->N2')
    assert 10.5 <= seen['replace'] <= 11.5, '막힌 지 10초 뒤 보고'
    assert 0.0 < seen['x'] < 1.0, '막혀서 N1과 N2 사이에 서 있었음'
    assert reached(r) == ['N1', 'N3', 'N4'], 'N1으로 돌아가 N3 -> N4'
    assert [n for t, n, _ in r.events if t == CMD_DONE] == ['N4']
    assert not d.holding and len([e for e in r.events if e[0] == EV_BLOCKED]) == 1


def test_no_detour_keeps_waiting_then_continues_when_clear():
    # 서버가 다른 길이 없어 아무것도 안 보냄 -> 계속 서 있다가 비키면 원래 길
    r, d = make()
    d.on_move(1, False, FORWARD, [N2], False, 'N2')

    def hook(t):
        d.set_obstacle('STATIC' if 1.0 <= t < 25.0 else '')
    run(r, d, 40, hook=hook)
    assert len([e for e in r.events if e[0] == EV_BLOCKED]) == 1
    assert reached(r) == ['N2'] and math.hypot(r.x - 1.0, r.y) < 1e-6

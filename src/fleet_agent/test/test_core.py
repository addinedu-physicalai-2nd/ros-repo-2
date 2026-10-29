"""core.Driver를 ROS 없이 가짜 로봇으로 돌려 본다:  python3 -m pytest test/  (또는 python3 test/test_core.py)"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet_agent.core import (CMD_DONE, CMD_FAILED, EV_BLOCKED, FORWARD, NODE_REACHED, REVERSE, BlockPolicy, Driver,  # noqa: E402
                              ObstacleClass, ReverseControl, wrap)

STOPS = {
    'N14': {'method': 'align', 'markers': [5, 6]},
    'D1': {'method': 'dock', 'marker': 1, 'back': 0.0},
    'D2': {'method': 'dock', 'marker': 2, 'back': 0.0},
    'D3': {'method': 'dock', 'marker': 3, 'back': 0.1},
    'P1': {'method': 'tape'},
}


class FakeRobot:
    """Nav2는 직선으로 목표까지 가서 yaw를 맞추고, cmd_vel은 적분. 정밀 정차는 다음 tick에 ok"""

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.nav = None
        self.events = []
        self.prec = []
        self.prec_pending = False
        self.dirs = []
        self.lcd = []
        self.driver = None
        self.prec_delay = 0      # 정밀 정차 결과를 이만큼 step 뒤에 (0 = 다음 step)
        self.prec_ok = True
        self.nav_fail = 0        # Nav2 목표를 이만큼 실패시킴
        self.prec_cancelled = 0
        self.guides = []         # 안내 요청 (action, node, 그때 yaw)
        self.guide_cancels = 0

    # io
    def nav_go(self, x, y, yaw):
        if self.nav_fail > 0:
            self.nav_fail -= 1
            self.nav = None
            self.driver.on_nav_result(False)
            return
        self.nav = (x, y, yaw)

    def nav_cancel(self):
        self.nav = None

    def cmd_vel(self, v, w):
        self.v, self.w = v, w

    def event(self, t, cmd, node, reason):
        self.events.append((t, node, reason))

    def precision(self, req):
        self.prec.append(req['node'] + ':' + req['method'])
        self.prec_pending = True
        self.prec_wait = self.prec_delay

    def precision_cancel(self):
        if self.prec_pending:
            self.prec_cancelled += 1
        self.prec_pending = False

    def guide(self, req):
        self.guides.append((req['action'], req['node'], self.yaw))

    def guide_cancel(self):
        self.guide_cancels += 1

    def direction(self, d):
        self.dirs.append(d)

    def lcd_override(self, text):
        self.lcd.append(text)

    def log(self, text):
        pass

    def step(self, dt):
        if self.nav:
            tx, ty, tyaw = self.nav
            d = math.hypot(tx - self.x, ty - self.y)
            sp = 0.15 * dt
            if d <= sp:
                self.x, self.y, self.yaw = tx, ty, tyaw
                self.nav = None
                self.driver.on_nav_result(True)
            else:
                self.yaw = math.atan2(ty - self.y, tx - self.x)
                self.x += (tx - self.x) / d * sp
                self.y += (ty - self.y) / d * sp
        else:
            self.yaw = wrap(self.yaw + self.w * dt)
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
        if self.prec_pending:
            if self.prec_wait > 0:
                self.prec_wait -= 1
            else:
                self.prec_pending = False
                self.driver.on_precision_result(self.prec_ok, 'fake fail' if not self.prec_ok else 'fake')


def run(robot, drv, secs, t0=0.0, dt=0.05, hook=None):
    t = t0
    while t < t0 + secs:
        if hook:
            hook(t)
        drv.tick(t, (robot.x, robot.y, robot.yaw))
        robot.step(dt)
        t += dt
    return t


def make(x, y, yaw, last_drive=None):
    r = FakeRobot(x, y, yaw)
    d = Driver(r, STOPS, reverse=ReverseControl(speed=0.1), policy=BlockPolicy())
    d.last_drive = last_drive
    r.driver = d
    return r, d


def reached(r):
    return [n for t, n, _ in r.events if t == NODE_REACHED]


def test_d3_exit_reverse_to_pickup():
    # D3에서 나와 후진으로 P1까지 (서버 로그: FORWARD [N16] -> REVERSE [P2] -> REVERSE [P1])
    r, d = make(3.548, 1.700, math.pi, last_drive=REVERSE)
    d.on_move(10, False, FORWARD, [('N16', 3.398, 1.700, -1.5708)])
    d.on_move(10, True, REVERSE, [('P2', 3.398, 2.100, -1.5708)])
    d.on_move(10, True, REVERSE, [('P1', 3.398, 2.600, 3.1416)])
    run(r, d, 40)
    assert reached(r) == ['N16', 'P2', 'P1']
    assert [n for t, n, _ in r.events if t == CMD_DONE] == ['P1']
    assert r.prec == ['P1:tape']
    assert abs(r.x - 3.398) < 0.04 and abs(r.y - 2.600) < 0.04
    assert abs(wrap(r.yaw - math.pi)) < 0.2          # P1에서 왼쪽을 보게 돌았다
    assert 'REVERSE' in r.dirs


def test_d2_pass_d1_forward_then_reverse():
    # D2로: D1까지 전진 -> 돌아서 D2로 후진 진입 -> dock
    r, d = make(1.698, 2.600, -math.pi / 2, last_drive=FORWARD)
    d.on_move(20, False, FORWARD, [('D1', 1.748, 1.525, -1.5708)])
    d.on_move(20, True, REVERSE, [('D2', 1.748, 1.375, 3.1416)])
    run(r, d, 40)
    assert reached(r) == ['D1', 'D2']
    assert r.prec == ['D2:dock']                     # D1은 지나가기만 (dock 안 함)
    assert [n for t, n, _ in r.events if t == CMD_DONE] == ['D2']


def test_late_append_waits_in_grace():
    # 줄이 빈 뒤 0.3초 만에 APPEND가 오면 끝 처리(CMD_DONE) 없이 이어 간다
    r, d = make(2.123, 2.600, 0.0, last_drive=FORWARD)
    d.on_move(30, False, FORWARD, [('T1', 2.348, 2.600, 0.0)])
    t = run(r, d, 1.6)
    assert reached(r) == ['T1']
    run(r, d, 0.3, t0=t)
    d.on_move(30, True, FORWARD, [('N3', 2.573, 2.600, 0.0)])
    run(r, d, 5, t0=t + 0.3)
    assert reached(r) == ['T1', 'N3']
    assert [n for t, n, _ in r.events if t == CMD_DONE] == ['N3']


def test_align_at_n14_then_continue():
    r, d = make(3.023, 1.200, 0.0, last_drive=FORWARD)
    d.on_move(40, False, FORWARD, [('N14', 3.398, 1.200, -1.5708)])
    d.on_move(40, True, REVERSE, [('N15', 3.398, 1.550, -1.5708)])
    run(r, d, 20)
    assert r.prec == ['N14:align']
    assert reached(r) == ['N14', 'N15']


def test_static_obstacle_blocks_then_resumes():
    r, d = make(3.398, 1.200, -math.pi / 2, last_drive=REVERSE)
    d.on_move(50, False, REVERSE, [('N15', 3.398, 1.550, -1.5708), ('N16', 3.398, 1.700, -1.5708)])

    def hook(t):
        d.set_obstacle('STATIC' if 1.0 <= t < 5.0 else '')
    run(r, d, 15, hook=hook)
    blocked = [e for e in r.events if e[0] == EV_BLOCKED]
    assert len(blocked) == 1 and blocked[0][2].startswith('STATIC')
    assert reached(r) == ['N15', 'N16']


def test_append_cancels_precision_at_pass_through_dock():
    # D2로 가는데 D2 lock이 늦어 D1에서 줄이 빔 -> D1을 끝으로 보고 dock 시작 -> APPEND(D2)가 오면 취소하고 이어 감
    r, d = make(1.698, 2.600, -math.pi / 2, last_drive=FORWARD)
    r.prec_delay = 10 ** 6  # 도킹이 끝나지 않는 중
    d.on_move(60, False, FORWARD, [('D1', 1.748, 1.525, -1.5708)])
    t = run(r, d, 12)
    assert r.prec == ['D1:dock'] and d.phase == 'precision'
    r.prec_delay = 0
    d.on_move(60, True, REVERSE, [('D2', 1.748, 1.375, 3.1416)])
    run(r, d, 15, t0=t)
    assert r.prec_cancelled == 1
    assert reached(r) == ['D1', 'D2']
    assert [n for tt, n, _ in r.events if tt == CMD_DONE] == ['D2']


def test_precision_fail_reason_in_cmd_done():
    r, d = make(3.548, 1.700, math.pi, last_drive=REVERSE)
    r.prec_ok = False
    d.on_move(70, False, FORWARD, [('N16', 3.398, 1.700, -1.5708)])
    d.on_move(70, True, REVERSE, [('D3', 3.548, 1.700, 3.1416)])
    run(r, d, 20)
    done = [(n, why) for tt, n, why in r.events if tt == CMD_DONE]
    assert done == [('D3', 'D3:fake fail')]


def test_nav_retry_then_ok():
    r, d = make(2.123, 2.600, 0.0, last_drive=FORWARD)
    r.nav_fail = 1  # 첫 목표만 실패
    d.on_move(80, False, FORWARD, [('T1', 2.348, 2.600, -1.5708)])
    run(r, d, 5)
    assert reached(r) == ['T1'] and not d.error


def test_obstacle_class():
    c = ObstacleClass()
    assert c.update(0.0, 0.15) == 'UNKNOWN'
    for i in range(1, 30):  # 0.1초마다, 거리 그대로
        k = c.update(i * 0.1, 0.15)
    assert k == 'STATIC'
    c.reset()
    k = ''
    for i in range(30):  # 사람: 거리가 흔들림
        k = c.update(i * 0.1, 0.15 + 0.05 * math.sin(i))
    assert k == 'DYNAMIC'
    assert c.update(5.0, None) == ''


def test_block_policy_dynamic():
    p = BlockPolicy(dynamic_wait=3, dynamic_ask=5)
    asks, reports = [], []
    for i in range(100):
        a, rep = p.update(i * 0.1, 'DYNAMIC')
        asks.append(a)
        if rep:
            reports.append(i * 0.1)
    assert not asks[0] and asks[40] and len(reports) == 1 and abs(reports[0] - 8.0) < 0.15




# ---------------------------------------------------------------- 안내 (Move.guide)
def auto_ok(r, d, ok=True):
    """Guide 노드 흉내: greet / watch 요청이 오면 바로 결과"""
    def hook(t):
        if d.guide_state == 'running' and r.guides and r.guides[-1][0] in ('greet', 'watch'):
            d.on_guide_result(ok, 'guest_seen' if ok else 'not_seen')
    return hook


def test_guide_greet_first_then_arrive_effect():
    r, d = make(0.0, 0.0, 0.0, last_drive=FORWARD)
    d.last_node = 'D1'
    d.on_move(60, False, FORWARD, [('N1', 1.0, 0.0, 0.0)], True, 'T1')
    t = run(r, d, 2)
    assert r.guides == [('greet', 'D1', 0.0)]       # 떠나기 전에 손님 기다림
    assert abs(r.x) < 1e-6 and reached(r) == []
    d.on_guide_result(True, 'guest_seen')
    d.on_move(60, True, FORWARD, [('T1', 2.0, 0.0, 0.0)], True, 'T1')
    run(r, d, 20, t0=t)
    assert reached(r) == ['N1', 'T1']                # 곧은 길: 확인 없이 지나감
    assert [g[0] for g in r.guides] == ['greet', 'arrive']
    assert [n for t_, n, _ in r.events if t_ == CMD_DONE] == ['T1']


def test_guide_checks_at_corner_only():
    r, d = make(0.0, 0.0, 0.0, last_drive=FORWARD)
    d.on_move(61, False, FORWARD, [('N1', 1.0, 0.0, 0.0), ('N2', 2.0, 0.0, 0.0), ('N3', 2.0, 1.0, 1.5708)],
              True, 'N3')
    yaw_at_watch = []
    ok = auto_ok(r, d)

    def hook(t):
        if d.phase == 'guide_watch' and not yaw_at_watch:
            yaw_at_watch.append((r.x, r.yaw))
        ok(t)
    run(r, d, 40, hook=hook)
    assert [g[:2] for g in r.guides] == [('greet', 'N1'), ('watch', 'N2'), ('arrive', 'N3')]  # N1 곧음, N2 꺾임
    x, yaw = yaw_at_watch[0]
    assert abs(x - 2.0) < 0.01 and abs(wrap(yaw - math.pi)) < 0.2   # N2에 서서 뒤(왔던 쪽)를 봄
    assert reached(r) == ['N1', 'N2', 'N3']


def test_guide_guest_lost_stops_and_reports():
    r, d = make(0.0, 0.0, 0.0, last_drive=FORWARD)
    d.on_move(62, False, FORWARD, [('N1', 1.0, 0.0, 0.0), ('N2', 1.0, 1.0, 1.5708)], True, 'N9')
    ok = auto_ok(r, d)

    def hook(t):
        if r.guides and r.guides[-1][0] == 'watch' and d.guide_state == 'running':
            d.on_guide_result(False, 'not_seen')
        else:
            ok(t)
    run(r, d, 30, hook=hook)
    fails = [(n, why) for t_, n, why in r.events if t_ == CMD_FAILED]
    assert fails == [('N1', 'guest_lost:N1:not_seen')]
    assert reached(r) == ['N1']
    assert d.phase == 'idle' and not d.error and len(d.queue) == 0 and not d.in_guide


def test_guide_resend_does_not_greet_again_and_serve_never_greets():
    r, d = make(0.0, 0.0, 0.0, last_drive=FORWARD)
    d.on_move(63, False, FORWARD, [('N1', 1.0, 0.0, 0.0)], True, 'N5')
    run(r, d, 15, hook=auto_ok(r, d))
    d.on_move(64, False, FORWARD, [('N2', 2.0, 0.0, 0.0)], True, 'N5')      # 통신 끊김 뒤 다시 보냄 (새 REPLACE)
    run(r, d, 15, t0=15, hook=auto_ok(r, d))
    assert [g[0] for g in r.guides] == ['greet']
    d.on_move(65, False, FORWARD, [('N3', 3.0, 0.0, 0.0)], False, 'N3')     # 안내 끝 -> 복귀
    run(r, d, 15, t0=30)
    assert [g[0] for g in r.guides] == ['greet'] and not d.in_guide
    assert reached(r) == ['N1', 'N2', 'N3']


def test_guide_replace_during_watch_cancels():
    r, d = make(0.0, 0.0, 0.0, last_drive=FORWARD)
    d.on_move(66, False, FORWARD, [('N1', 1.0, 0.0, 0.0), ('N2', 1.0, 1.0, 1.5708)], True, 'N9')
    ok = auto_ok(r, d)

    def hook(t):
        if not (r.guides and r.guides[-1][0] == 'watch'):
            ok(t)
    t = run(r, d, 20, hook=hook)
    assert d.phase == 'guide_watch'
    d.on_move(67, False, FORWARD, [('N0', 0.0, 0.0, 3.14)], False, 'N0')
    assert r.guide_cancels == 1
    run(r, d, 15, t0=t)
    assert reached(r)[-1] == 'N0'


class LeaveRobot(FakeRobot):
    """leave 정밀 동작 흉내: undock은 N15에서 남쪽(축)을 보게 옮겨 놓고, Nav2 호출 수를 센다"""

    def __init__(self, *a):
        super().__init__(*a)
        self.nav_calls = []
        self.teleport = None

    def nav_go(self, x, y, yaw):
        self.nav_calls.append((round(x, 3), round(y, 3)))
        super().nav_go(x, y, yaw)

    def precision(self, req):
        super().precision(req)
        if req['method'] == 'undock':
            self.teleport = (3.398, 1.550, -math.pi / 2 + 0.05)   # 마커로 맞춘 축 (지도와 약간 다름)
        if req['method'] == 'tape' and req.get('approach'):
            self.approach_seen = True
            self.teleport = (3.398, 2.640, self.yaw)              # tape가 가로선까지 (좌표보다 4 cm 더)

    def step(self, dt):
        fired = self.prec_pending and self.prec_wait <= 0
        if fired and self.teleport:
            self.x, self.y, self.yaw = self.teleport
            self.teleport = None
        super().step(dt)


def test_d4_leave_undock_axis_reverse_tape_then_tape_exit():
    stops = dict(STOPS)
    stops['D4'] = {'method': 'dock', 'marker': 2, 'back': 0.1,
                   'leave': {'method': 'undock', 'to': 'N15', 'hold_axis': True}}
    stops['P1'] = {'method': 'tape', 'leave': {'method': 'tape_exit', 'quiet_to_next': True}}
    r = LeaveRobot(3.548, 1.550, math.pi)
    d = Driver(r, stops, reverse=ReverseControl(speed=0.1), policy=BlockPolicy())
    r.driver = d
    d.last_node, d.leave_from, d.last_drive = 'D4', 'D4', REVERSE
    d.on_move(70, False, FORWARD, [('N15', 3.398, 1.550, -1.5708)])
    d.on_move(70, True, REVERSE, [('P2', 3.398, 2.100, -1.5708)])
    d.on_move(70, True, REVERSE, [('P1', 3.398, 2.600, 3.1416)])
    t = run(r, d, 40)
    assert r.prec == ['D4:undock', 'P1:tape']          # D4를 떠날 때 undock, P1에서 tape
    assert r.nav_calls == []                           # N15까지 Nav2 안 씀
    assert reached(r) == ['N15', 'P2', 'P1']
    assert [n for tt, n, _ in r.events if tt == CMD_DONE] == ['P1']
    assert abs(wrap(r.yaw - (-math.pi / 2 + 0.05))) < 0.05   # 축 그대로 (출발 회전 · P1 yaw 회전 안 함)
    assert getattr(r, 'approach_seen', False)          # P2 -> P1은 좌표 대신 tape가 가로선으로
    assert abs(r.y - 2.640) < 0.01
    d.on_move(71, False, FORWARD, [('N4', 3.0, 2.6, 3.1416)])

    def wall(tt):
        d.set_obstacle('STATIC')          # P1 -> N4 좁은 틈: LiDAR엔 벽이 가깝게 보임
    run(r, d, 20, t0=t, hook=wall)
    assert not [e for e in r.events if e[0] == EV_BLOCKED]   # quiet_to_next라 N4까지 안 멈춤
    assert r.prec == ['D4:undock', 'P1:tape', 'P1:tape_exit']   # P1을 떠날 때 테이프 빼기 먼저
    assert r.nav_calls and r.nav_calls[0] == (3.0, 2.6)
    assert reached(r)[-1] == 'N4'


def test_d4_flow_with_goal_node_v9():
    # 같은 흐름을 v9 서버처럼 goal_node를 붙여 보냄 (도착지 P1, 다음 이동 도착지 N4)
    stops = dict(STOPS)
    stops['D4'] = {'method': 'dock', 'marker': 2, 'back': 0.1,
                   'leave': {'method': 'undock', 'to': 'N15', 'hold_axis': True}}
    stops['P1'] = {'method': 'tape', 'leave': {'method': 'tape_exit', 'quiet_to_next': True}}
    r = LeaveRobot(3.548, 1.550, math.pi)
    d = Driver(r, stops, reverse=ReverseControl(speed=0.1), policy=BlockPolicy())
    r.driver = d
    d.last_node, d.leave_from, d.last_drive = 'D4', 'D4', REVERSE
    d.on_move(70, False, FORWARD, [('N15', 3.398, 1.550, -1.5708)], False, 'P1')
    d.on_move(70, True, REVERSE, [('P2', 3.398, 2.100, -1.5708)], False, 'P1')
    d.on_move(70, True, REVERSE, [('P1', 3.398, 2.600, 3.1416)], False, 'P1')
    t = run(r, d, 40)
    assert r.prec == ['D4:undock', 'P1:tape']          # D4를 떠날 때 undock, P1에서 tape
    assert r.nav_calls == []                           # N15까지 Nav2 안 씀
    assert reached(r) == ['N15', 'P2', 'P1']
    assert [n for tt, n, _ in r.events if tt == CMD_DONE] == ['P1']
    assert abs(wrap(r.yaw - (-math.pi / 2 + 0.05))) < 0.05   # 축 그대로 (출발 회전 · P1 yaw 회전 안 함)
    assert getattr(r, 'approach_seen', False)          # P2 -> P1은 좌표 대신 tape가 가로선으로
    assert abs(r.y - 2.640) < 0.01
    d.on_move(71, False, FORWARD, [('N4', 3.0, 2.6, 3.1416)], False, 'T1')

    def wall(tt):
        d.set_obstacle('STATIC')          # P1 -> N4 좁은 틈: LiDAR엔 벽이 가깝게 보임
    run(r, d, 20, t0=t, hook=wall)
    assert not [e for e in r.events if e[0] == EV_BLOCKED]   # quiet_to_next라 N4까지 안 멈춤
    assert r.prec == ['D4:undock', 'P1:tape', 'P1:tape_exit']   # P1을 떠날 때 테이프 빼기 먼저
    assert r.nav_calls and r.nav_calls[0] == (3.0, 2.6)
    assert reached(r)[-1] == 'N4'


def test_goal_node_no_precision_when_queue_end_is_not_goal():
    # 서버가 lock 때문에 D1까지만 보냄 (도착지는 D2) -> D1에서 도킹하지 않고 CMD_DONE만
    r, d = make(1.698, 2.600, -math.pi / 2, last_drive=FORWARD)
    d.on_move(70, False, FORWARD, [('D1', 1.748, 1.525, -1.5708)], False, 'D2')
    run(r, d, 15)
    assert r.prec == [] and [n for t, n, _ in r.events if t == CMD_DONE] == ['D1']
    assert d.leave_from == ''                         # 도킹 안 했으니 leave 준비도 없음


def test_goal_node_precision_at_goal():
    r, d = make(1.698, 2.600, -math.pi / 2, last_drive=FORWARD)
    d.on_move(71, False, FORWARD, [('D1', 1.748, 1.525, -1.5708)], False, 'D1')
    run(r, d, 15)
    assert r.prec == ['D1:dock']


def test_guide_goal_skips_precision_and_plays_arrive():
    # 안내 도착지가 정밀 정차 노드여도 정밀 정차 없이 Nav2 위치로 서고 도착 효과
    r, d = make(1.698, 2.600, -math.pi / 2, last_drive=FORWARD)
    d.on_move(72, False, FORWARD, [('D1', 1.748, 1.525, -1.5708)], True, 'D1')
    run(r, d, 15, hook=auto_ok(r, d))
    assert r.prec == []
    assert [g[0] for g in r.guides] == ['greet', 'arrive']


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)

"""fleet_agent 핵심 로직 (ROS를 모른다. agent_node가 ROS 입출력을 이어 준다)

- StepQueue      : 서버 Move를 waypoint 하나씩(Step)으로 펼쳐 둔 줄
- ReverseControl : cmd_vel 직선 후진 (위치 · 방향 P제어)
- BlockPolicy    : 장애물로 멈췄을 때 언제 서버에 BLOCKED를 보낼지 (사람 / 물건)
- ObstacleClass  : LiDAR 거리 변화로 사람(DYNAMIC) / 물건(STATIC) 구분
- Driver         : 노드 단위 주행 상태기계 (fleet_msgs README 5장 로봇 규칙)
"""
import math
from dataclasses import dataclass

import yaml

FORWARD = 0
REVERSE = 1

# RobotState.state
IDLE, MOVING, PAUSED, BLOCKED, ERROR = 0, 1, 2, 3, 4
# RobotEvent.type
NODE_REACHED, CMD_DONE, CMD_FAILED, EV_BLOCKED = 0, 1, 2, 4
# Control.command
PAUSE, RESUME, CANCEL = 0, 1, 2


def wrap(a):
    """각도를 -pi ~ pi 로"""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lim):
    return max(-lim, min(lim, v))


def load_precision_stops(path):
    """fleet_msgs/docs/precision_stops.yaml -> {node_id: {method, marker, markers, back, ...}}"""
    if not path:
        return {}
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    out = {}
    for node, cfg in data.items():
        if isinstance(cfg, dict) and cfg.get('method') in ('dock', 'align', 'tape'):
            out[str(node)] = dict(cfg)
    return out


@dataclass
class Step:
    cmd_id: int
    drive: int
    node_id: str
    x: float
    y: float
    yaw: float


class StepQueue:
    """REPLACE = 비우고 새 cmd_id로, APPEND = 지금 cmd_id와 같을 때만 뒤에 이어 붙인다"""

    def __init__(self):
        self.steps = []
        self.cmd_id = 0

    def apply(self, cmd_id, append, drive, waypoints):
        if append:
            if cmd_id != self.cmd_id:
                return False  # 다른 명령에 붙이려는 APPEND는 버린다
        else:
            self.steps.clear()
            self.cmd_id = cmd_id
        for w in waypoints:
            self.steps.append(Step(cmd_id, drive, w[0], w[1], w[2], w[3]))
        return True

    def clear(self):
        self.steps.clear()

    def __len__(self):
        return len(self.steps)


class ReverseControl:
    """뒤가 진행 방향이 되게 (tx, ty)까지 직선 후진"""

    def __init__(self, speed=0.06, k_ang=1.8, max_w=0.8, tol=0.03, turn_first=0.35):
        self.speed = speed
        self.k_ang = k_ang
        self.max_w = max_w
        self.tol = tol
        self.turn_first = turn_first  # 방향 오차가 이보다 크면 먼저 제자리 회전

    def command(self, pose, target, start=None):
        """-> (v, w, done). start를 주면 목표를 지나쳤는지도 본다"""
        x, y, yaw = pose
        tx, ty = target
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < self.tol:
            return 0.0, 0.0, True
        if start is not None:  # 출발점 -> 목표 방향으로 목표를 지나갔으면 도착
            sx, sy = start
            if (tx - sx) * (x - tx) + (ty - sy) * (y - ty) > 0.0:
                return 0.0, 0.0, True
        face = wrap(math.atan2(dy, dx) + math.pi)  # 로봇 앞이 볼 방향 (뒤로 가니까 반대)
        err = wrap(face - yaw)
        w = clamp(self.k_ang * err, self.max_w)
        if abs(err) > self.turn_first:
            return 0.0, w, False
        v = -min(self.speed, max(0.02, d)) * math.cos(err)
        return v, w, False


def turn_command(yaw, target, k=1.8, max_w=0.8, tol=0.08):
    """제자리 회전 -> (w, done)"""
    err = wrap(target - yaw)
    if abs(err) < tol:
        return 0.0, True
    w = clamp(k * err, max_w)
    if abs(w) < 0.15:
        w = math.copysign(0.15, w)  # 너무 느리면 안 돈다
    return w, False


class ObstacleClass:
    """막힌 동안 앞 거리 변화로 종류를 가른다 (로봇은 멈춰 있다고 본다)
    - 거리가 window 동안 move_thresh 넘게 흔들리면 DYNAMIC (사람)
    - static_sec 동안 그대로면 STATIC (물건)
    - 그 전에는 UNKNOWN
    - person=True (카메라 YOLO)면 바로 DYNAMIC
    """

    def __init__(self, window=1.5, move_thresh=0.04, static_sec=2.0, settle_sec=0.5):
        self.window = window
        self.move_thresh = move_thresh
        self.static_sec = static_sec
        self.settle_sec = settle_sec  # 막힌 직후 로봇이 서는 동안은 거리 변화를 안 본다
        self.since = None
        self.hist = []
        self.dynamic_seen = False

    def reset(self):
        self.since = None
        self.hist = []
        self.dynamic_seen = False

    def update(self, t, dist, person=False):
        """dist = 앞 장애물 거리 (None = 막힘 없음) -> kind 또는 '' (막힘 없음)"""
        if dist is None:
            self.reset()
            return ''
        if self.since is None:
            self.since = t
        if t - self.since >= self.settle_sec:
            self.hist.append((t, dist))
            self.hist = [(ht, hd) for ht, hd in self.hist if t - ht <= self.window]
        if person:
            self.dynamic_seen = True
        if len(self.hist) >= 2:
            ds = [hd for _, hd in self.hist]
            if max(ds) - min(ds) > self.move_thresh:
                self.dynamic_seen = True
        if self.dynamic_seen:
            return 'DYNAMIC'
        if t - self.since >= self.static_sec:
            return 'STATIC'
        return 'UNKNOWN'


class BlockPolicy:
    """멈춘 뒤 서버에 BLOCKED를 보낼 때 (README 장애물 단계)
    - 사람(DYNAMIC): dynamic_wait 초 기다림 -> dynamic_ask 초 동안 LCD '지나갈게요' -> 보고
    - 물건(STATIC) : static_wait 초 확인 -> 보고
    - 모름(UNKNOWN): unknown_wait 초 뒤 보고
    보고는 한 번만. 길이 비면 처음부터.
    """

    def __init__(self, dynamic_wait=3.0, dynamic_ask=5.0, static_wait=2.0, unknown_wait=8.0):
        self.dynamic_wait = dynamic_wait
        self.dynamic_ask = dynamic_ask
        self.static_wait = static_wait
        self.unknown_wait = unknown_wait
        self.since = None
        self.reported = False

    def update(self, t, kind):
        """kind '' = 막힘 없음 -> (ask, report_kind 또는 None)"""
        if not kind:
            self.since = None
            self.reported = False
            return False, None
        if self.since is None:
            self.since = t
        el = t - self.since
        ask = False
        due = None
        if kind == 'DYNAMIC':
            ask = self.dynamic_wait <= el < self.dynamic_wait + self.dynamic_ask
            due = self.dynamic_wait + self.dynamic_ask
        elif kind == 'STATIC':
            due = self.static_wait
        else:
            due = self.unknown_wait
        if not self.reported and el >= due:
            self.reported = True
            return ask, kind
        return ask, None


class Driver:
    """노드 단위 주행 상태기계

    io (agent_node가 넘겨줌)
      nav_go(x, y, yaw)        Nav2 목표 보내기 (결과는 on_nav_result)
      nav_cancel()
      cmd_vel(v, w)
      event(type, cmd_id, node, reason)
      precision(req)           Docking 노드에 요청 (결과는 on_precision_result)
      precision_cancel()
      direction(text)          Obstacle 노드에 지금 볼 방향 (FORWARD / REVERSE / STOP)
      lcd_override(text)       장애물 안내 문구 (None = 서버 문구로 돌아감)
      log(text)

    규칙 (fleet_msgs README 5장)
      - 받은 노드까지만 간다. 노드마다 NODE_REACHED
      - 같은 방향 APPEND는 멈추지 않고 이어서, drive가 바뀌면 그 노드에서 멈추고 제자리 회전 후 출발
      - FORWARD = Nav2, REVERSE = cmd_vel 직선 후진
      - 중간 노드는 위치만 통과, 줄 끝 노드에서만 yaw를 맞춘다
      - 줄 끝 노드가 dock / tape 정밀 정차면 끝낸 뒤 CMD_DONE. align 노드는 지날 때마다 맞추고 이어 간다
    """

    def __init__(self, io, stops, pass_tol=0.08, yaw_tol=0.15, done_grace=0.5, reverse=None, policy=None,
                 nav_retries=1):
        self.io = io
        self.stops = stops
        self.pass_tol = pass_tol      # 중간 노드 통과 판정 거리 (다음 노드로 바로 넘어감)
        self.yaw_tol = yaw_tol
        self.done_grace = done_grace  # 줄이 비고 이만큼 기다렸다가 끝 처리 (연달아 오는 APPEND를 기다림)
        self.rev = reverse or ReverseControl()
        self.policy = policy or BlockPolicy()
        self.queue = StepQueue()
        self.cur = None
        self.phase = 'idle'           # idle / turn / nav / reverse / align / grace / final_turn / precision
        self.after_turn = ''          # turn 끝나고: move / finish
        self.turn_target = 0.0
        self.seg_start = None
        self.last_drive = None        # 마지막으로 달린 방향 (방향 바뀜 판단)
        self.last_node = ''
        self.paused = False
        self.error = False
        self.nav_state = ''           # '' / running / ok / fail
        self.prec_state = ''          # '' / running / ok
        self.grace_until = 0.0
        self.obstacle = ''            # 지금 진행 방향 장애물 종류 ('' = 없음)
        self.holding = False          # 장애물로 멈춘 중
        self.now = 0.0
        self.nav_retries = nav_retries  # Nav2 실패 때 다시 보내는 횟수 (그래도 안 되면 CMD_FAILED)
        self.nav_tries = 0
        self.prec_detail = ''         # 정밀 정차 실패 이유 -> CMD_DONE.reason

    # ------------------------------------------------------------ 입력
    def on_move(self, cmd_id, append, drive, waypoints):
        if not waypoints:
            return
        if not append:  # REPLACE: 지금 가던 것까지 버리고 새로
            self._stop_motion()
            self.queue.apply(cmd_id, False, drive, waypoints)
            self.cur = None
            self.phase = 'idle'
            self.error = False
            return
        if not self.queue.apply(cmd_id, True, drive, waypoints):
            self.io.log(f'다른 명령({cmd_id})에 붙이는 APPEND 무시 (지금 {self.queue.cmd_id})')
            return
        if self.phase == 'precision':
            # 끝 노드인 줄 알고 정밀 정차를 시작했는데 길이 더 왔다 = 지나가는 노드였다 -> 그만두고 이어 감
            self.io.log(f'{self.cur.node_id}: 길이 더 와서 정밀 정차 취소')
            self.io.precision_cancel()
            self.prec_state = ''
            self.phase = 'grace'
            self.grace_until = self.now

    def on_control(self, cmd):
        if cmd == PAUSE:
            self.paused = True
            self._stop_motion(keep=True)
        elif cmd == RESUME:
            self.paused = False
            self.error = False
            self._resume()
        elif cmd == CANCEL:
            self._stop_motion()
            self.queue.clear()
            self.cur = None
            self.phase = 'idle'
            self.paused = False

    def on_nav_result(self, ok):
        if self.phase == 'nav' and self.nav_state == 'running':
            self.nav_state = 'ok' if ok else 'fail'

    def on_precision_result(self, ok, detail=''):
        if self.phase in ('align', 'precision') and self.prec_state == 'running':
            self.prec_state = 'ok'
            if not ok:
                self.io.log(f'정밀 정차 실패, Nav2 위치로 진행: {detail}')
                self.prec_detail = f'{self.cur.node_id if self.cur else ""}:{detail}'

    def set_obstacle(self, kind):
        self.obstacle = kind or ''

    # ------------------------------------------------------------ 상태
    def run_state(self):
        if self.error:
            return ERROR
        if self.paused:
            return PAUSED
        if self.holding:
            return BLOCKED
        if self.phase == 'idle':
            return IDLE
        return MOVING

    # ------------------------------------------------------------ 주기 처리
    def tick(self, now, pose):
        self.now = now
        if pose is None or self.paused or self.error:
            return
        moving = self.phase in ('nav', 'reverse')
        if moving or self.holding:
            ask, report = self.policy.update(now, self.obstacle)
            if self.obstacle:
                if not self.holding:
                    self.holding = True
                    self._stop_motion(keep=True, keep_dir=True)  # 방향은 그대로 둬야 Obstacle 노드가 계속 본다
                    self.io.log(f'장애물({self.obstacle}) -> 정지')
                self.io.lcd_override('지나갈게요' if ask else None)
                if report:
                    nxt = self.cur.node_id if self.cur else ''
                    self.io.event(EV_BLOCKED, self.queue.cmd_id, nxt, f'{report}:{self.last_node}->{nxt}')
                return
            if self.holding:  # 길이 비었다
                self.holding = False
                self.io.lcd_override(None)
                self.io.log('장애물 없어짐 -> 다시 출발')
                self._resume()
                return

        if self.phase == 'idle':
            if len(self.queue):
                self._start_next(pose)
        elif self.phase == 'turn' or self.phase == 'final_turn':
            w, done = turn_command(pose[2], self.turn_target, tol=self.yaw_tol)
            self.io.cmd_vel(0.0, w)
            if done:
                self.io.cmd_vel(0.0, 0.0)
                if self.phase == 'turn':
                    self._begin_move(pose)
                else:
                    self._precision_or_done()
        elif self.phase == 'nav':
            self._tick_nav(pose)
        elif self.phase == 'reverse':
            v, w, done = self.rev.command(pose, (self.cur.x, self.cur.y), self.seg_start)
            if done:
                nxt = self.queue.steps[0] if len(self.queue) else None
                if not (nxt and nxt.drive == REVERSE):
                    self.io.cmd_vel(0.0, 0.0)
                self._reached(pose)
            else:
                self.io.cmd_vel(v, w)
        elif self.phase == 'align' or self.phase == 'precision':
            if self.prec_state == 'ok':
                self.prec_state = ''
                if self.phase == 'align':
                    self._after_node(pose)
                else:
                    self._done()
        elif self.phase == 'grace':
            if len(self.queue):
                self._start_next(pose)
            elif now >= self.grace_until:
                self._finish(pose)

    # ------------------------------------------------------------ 내부
    def _tick_nav(self, pose):
        d = math.hypot(self.cur.x - pose[0], self.cur.y - pose[1])
        nxt = self.queue.steps[0] if len(self.queue) else None
        if nxt and nxt.drive == FORWARD and d < self.pass_tol:
            self._reached(pose)  # 중간 노드: 위치만 통과, 다음 목표로 바로
            return
        if self.nav_state == 'ok':
            self._reached(pose)
        elif self.nav_state == 'fail':
            if self.nav_tries < self.nav_retries:  # 한 번 더 보내 본다
                self.nav_tries += 1
                self.io.log(f'Nav2 실패: {self.cur.node_id} -> 다시 시도 ({self.nav_tries}/{self.nav_retries})')
                self.nav_state = 'running'
                self.io.nav_go(self.cur.x, self.cur.y, self.cur.yaw)
                return
            self.io.log(f'Nav2 실패: {self.cur.node_id}')
            self.io.event(CMD_FAILED, self.queue.cmd_id, self.cur.node_id, 'nav2_failed')
            self.error = True
            self.phase = 'idle'
            self.io.direction('STOP')

    def _start_next(self, pose):
        self.cur = self.queue.steps.pop(0)
        if self.last_drive is not None and self.cur.drive != self.last_drive:
            # 방향이 바뀜: 이 자리(lock 노드)에서 멈추고 돈다
            heading = math.atan2(self.cur.y - pose[1], self.cur.x - pose[0])
            self.turn_target = wrap(heading + math.pi) if self.cur.drive == REVERSE else heading
            self.phase = 'turn'
            self.io.direction('STOP')
            return
        self._begin_move(pose)

    def _begin_move(self, pose):
        self.seg_start = (pose[0], pose[1])
        if self.cur.drive == FORWARD:
            self.phase = 'nav'  # nav_go 결과가 바로 와도 받을 수 있게 먼저
            self.nav_state = 'running'
            self.nav_tries = 0
            self.io.direction('FORWARD')
            self.io.nav_go(self.cur.x, self.cur.y, self.cur.yaw)
        else:
            self.phase = 'reverse'
            self.io.direction('REVERSE')

    def _resume(self):
        if self.cur is None:
            return
        if self.phase == 'nav':
            self.nav_state = 'running'
            self.io.nav_go(self.cur.x, self.cur.y, self.cur.yaw)
        elif self.phase == 'precision' or self.phase == 'align':
            self._request_precision(self.phase == 'align')

    def _reached(self, pose):
        step = self.cur
        self.last_node = step.node_id
        self.last_drive = step.drive
        self.nav_state = ''
        self.io.event(NODE_REACHED, step.cmd_id, step.node_id, '')
        stop = self.stops.get(step.node_id)
        if stop and stop['method'] == 'align':
            self.io.cmd_vel(0.0, 0.0)
            self.io.nav_cancel()
            self.phase = 'align'
            self.io.direction('STOP')
            self._request_precision(True)
            return
        self._after_node(pose)

    def _after_node(self, pose):
        if len(self.queue):
            self._start_next(pose)
            return
        self.phase = 'grace'
        self.grace_until = self.now + self.done_grace

    def _finish(self, pose):
        """줄 끝 노드: yaw 맞추기 -> (dock / tape) -> CMD_DONE"""
        self.io.direction('STOP')
        if self.cur.drive == REVERSE and abs(wrap(self.cur.yaw - pose[2])) > self.yaw_tol:
            self.turn_target = self.cur.yaw  # FORWARD는 Nav2가 이미 맞춤
            self.phase = 'final_turn'
            return
        self._precision_or_done()

    def _precision_or_done(self):
        stop = self.stops.get(self.cur.node_id)
        if stop and stop['method'] in ('dock', 'tape'):
            self.phase = 'precision'
            self._request_precision(False)
            return
        self._done()

    def _request_precision(self, align):
        stop = dict(self.stops.get(self.cur.node_id, {}))
        stop['node'] = self.cur.node_id
        self.prec_state = 'running'
        self.io.precision(stop)

    def _done(self):
        # 정밀 정차를 못 했으면 이유를 reason에 (서버가 경고만 띄우고 작업은 이어 감)
        self.io.event(CMD_DONE, self.queue.cmd_id, self.cur.node_id, self.prec_detail)
        self.prec_detail = ''
        self.phase = 'idle'
        self.io.direction('STOP')

    def _stop_motion(self, keep=False, keep_dir=False):
        self.io.cmd_vel(0.0, 0.0)
        self.io.nav_cancel()
        if self.phase in ('align', 'precision'):
            self.io.precision_cancel()
        if not keep:
            self.nav_state = ''
            self.prec_state = ''
            self.holding = False
            self.policy.update(0.0, '')
            self.io.lcd_override(None)
        if not keep_dir:
            self.io.direction('STOP')

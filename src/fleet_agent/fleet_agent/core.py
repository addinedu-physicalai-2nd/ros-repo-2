"""fleet_agent 핵심 로직 (ROS를 모른다. agent_node가 ROS 입출력을 이어 준다)

- StepQueue      : 서버 Move를 waypoint 하나씩(Step)으로 펼쳐 둔 줄
- ReverseControl : cmd_vel 직선 후진 (위치 · 방향 P제어). command_axis = 정해진 축(heading) 그대로 후진
- BlockPolicy    : 장애물로 멈췄을 때 언제 서버에 BLOCKED를 보낼지 (사람 / 물건)
- ObstacleClass  : LiDAR 거리 변화로 사람(DYNAMIC) / 물건(STATIC) 구분
- Driver         : 노드 단위 주행 상태기계 (fleet_msgs README 5장 로봇 규칙)
                   + 안내 (Move.guide: 출발 전 인사 · 꺾이는 노드에서 뒤돌아 확인 · 도착 효과, 손님 확인은 Guide 노드)
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
        if isinstance(cfg, dict) and cfg.get('method') in ('dock', 'align', 'tape', 'dock_in','table'):
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
    guide: bool = False   # Move.guide (안내 중)
    goal: str = ''        # Move.goal_node (최종 도착지, '' = 모름)


class StepQueue:
    """REPLACE = 비우고 새 cmd_id로, APPEND = 지금 cmd_id와 같을 때만 뒤에 이어 붙인다"""

    def __init__(self):
        self.steps = []
        self.cmd_id = 0

    def apply(self, cmd_id, append, drive, waypoints, guide=False, goal=''):
        if append:
            if cmd_id != self.cmd_id:
                return False  # 다른 명령에 붙이려는 APPEND는 버린다
        else:
            self.steps.clear()
            self.cmd_id = cmd_id
        for w in waypoints:
            self.steps.append(Step(cmd_id, drive, w[0], w[1], w[2], w[3], bool(guide), goal or ''))
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

    def command_axis(self, pose, target, axis, heading_err=None):
        """축 고정 후진: 로봇 앞 방향을 축으로 유지하고, 축 위로 잰 남은 거리만 본다 -> (v, w, done)
        (마커로 맞춘 축을 Nav2 좌표 쪽으로 다시 꺾지 않게. 좌우 위치는 축이 정한다)
        heading_err: 방향 오차를 odom으로 잰 값 (AMCL이 고칠 때마다 꺾이지 않게). 없으면 맵 yaw로"""
        x, y, yaw = pose
        b = axis + math.pi                      # 뒤로 가는 방향
        remain = (target[0] - x) * math.cos(b) + (target[1] - y) * math.sin(b)
        if remain < self.tol:
            return 0.0, 0.0, True
        err = heading_err if heading_err is not None else wrap(axis - yaw)
        w = clamp(self.k_ang * err, self.max_w)
        return -min(self.speed, max(0.02, remain)), w, False


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
      precision_extend(req)    통로 후진 중 길이 늘어남 (같은 요청에 nodes · then 갱신). 진행은 on_precision_progress
      direction(text)          Obstacle 노드에 지금 볼 방향 (FORWARD / REVERSE / STOP)
      lcd_override(text)       장애물 안내 문구 (None = 서버 문구로 돌아감)
      guide(req)               Guide 노드에 요청 {action: greet / watch / arrive} (결과는 on_guide_result)
      guide_cancel()
      log(text)

    규칙 (fleet_msgs README 5장)
      - 받은 노드까지만 간다. 노드마다 NODE_REACHED
      - 같은 방향 APPEND는 멈추지 않고 이어서, drive가 바뀌면 그 노드에서 멈추고 제자리 회전 후 출발
      - FORWARD = Nav2, REVERSE = cmd_vel 직선 후진
      - 중간 노드는 위치만 통과, 줄 끝 노드에서만 yaw를 맞춘다
      - 도착지(Move.goal_node)가 dock / tape 정밀 정차면 끝낸 뒤 CMD_DONE. align 노드는 지날 때마다 맞추고 이어 간다
        줄 끝이 도착지가 아니면(길이 막혀 서버가 거기까지만 보냄) 정밀 정차 없이 CMD_DONE하고 다음 Move를 기다린다
        goal_node가 비어 있으면(예전 서버) 줄 끝을 도착지로 본다
      - leave (precision_stops의 노드.leave): 그 노드에서 CMD_DONE 한 뒤(또는 켤 때 서 있던 노드) 다음 출발 전에
        docking에 맡긴다. 예) D3 leave=undock: 마커로 N15까지 나와 돌고 -> to 노드(N15) 도착으로 침
                              P1 leave=tape_exit: 테이프 선까지 앞으로 빼고 -> 그다음 Nav2
        hold_axis: 맞춘 방향을 축으로 이어지는 후진을 그 축 그대로 (출발 회전 · 끝 노드 yaw 회전 생략)
                   방향은 odom으로 유지 (io.odom_yaw가 있으면). 맵(AMCL) 보정에 따라 꺾이지 않는다
      - 축 고정 후진으로 tape 노드(P1)에 갈 때는 좌표까지 가지 않고 tape에 바로 맡긴다 (tape_approach):
        tape가 바닥 가로선을 보며 후진해 멈추고 삽입까지 -> NODE_REACHED · CMD_DONE (좌표가 조금 틀려도 됨)
      - 통로 넘겨받기 (corridor=True, 기본): 축이 정해진 뒤(N14 align hold_axis, undock hold_axis) 후진은
        cmd_vel로 직접 가지 않고 docking에 맡긴다 (corridor_approach, method=corridor, nodes = 받은 REVERSE 줄).
        docking이 라이다 양옆 벽과 평행하게 후진하며 바닥 가로선(N15 · N16)을 밟을 때마다 progress
          -> 그 노드 NODE_REACHED (멈추지 않음) -> 서버가 lock_lookahead만큼 다음 노드 APPEND
          -> APPEND(REVERSE)는 달리는 docking에 precision_extend로 이어 붙임 (안 섬)
          줄 안에 도착지가 있고 tape / dock_in이면 then으로 같이 맡김 (P1 삽입, 도크 줄 선에서 우90 -> 후면 주차)
        끝나면(result.reached) 그 노드에서 평소처럼 처리 (도착지면 CMD_DONE). 이어 붙이기가 늦어 docking이 먼저 섰으면
        못 간 노드는 큐로 돌려 거기서 다시 넘겨받기
        docking이 실패하거나 시간 초과면 도착으로 치지 않고 멈춤 -> CMD_FAILED reason="corridor_failed:<노드>:<이유>"
        (재개하면 남은 노드로 다시 맡김)
    안내 (Move.guide = true): 서버는 "안내 중"과 도착지만 알려 주고, 나머지는 로봇이 판단한다
      - 인사 : 안내가 아닌 상태에서 안내 Move를 처음 받으면, 떠나기 전에 Guide 'greet' (앞 카메라로 손님 기다림)
      - 확인 : 꺾이는 노드(들어온 방향과 나갈 방향이 check_turn 넘게 다름)에서 멈춤 -> 뒤로 돎 -> Guide 'watch'
               -> 다음 노드 쪽으로 돎 -> 이어 감. 바로 전 인사 · 확인 자리에서 check_gap m 안이면 건너뜀
      - 도착 : 도착지(goal_node)에서 CMD_DONE 뒤 Guide 'arrive' (도착 효과, 기다리지 않음). 안내는 정밀 정차 안 함
      - 손님을 못 찾으면 그 자리에 멈추고 CMD_FAILED reason="guest_lost:<노드>:<이유>" (고장 아님, 줄 비움)
    """

    def __init__(self, io, stops, pass_tol=0.08, yaw_tol=0.15, done_grace=0.5, reverse=None, policy=None,
                 nav_retries=1, check_turn_deg=45.0, check_gap=0.8, corridor=True):
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
        #                               guide_greet / guide_turn / guide_watch (안내)
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
        self.guide_state = ''         # '' / running / ok / fail
        self.guide_detail = ''
        self.guide_after = None       # guide 단계가 끝나면 할 일 (함수)
        self.in_guide = False         # 안내 중 (인사를 이미 함)
        self.check_turn = math.radians(check_turn_deg)
        self.check_gap = check_gap
        self.check_xy = None          # 마지막으로 인사 · 확인한 자리
        self.prev_xy = None           # 바로 전에 도착한 노드 (꺾임 판단)
        self.leave_from = ''          # 여기서 출발할 때 leave 정밀 동작을 먼저 (precision_stops의 leave)
        self.leave_req = None
        self.prec_ok = True
        self.axis = None              # 축 고정 후진 방향 (leave hold_axis로 정함, FORWARD 출발 때 해제)
        self.axis_odom = None         # 같은 축을 odom yaw로 (방향 유지는 이것으로)
        self.prec_done_node = ''      # tape_approach로 정밀 정차를 이미 끝낸 노드
        self.table_req = None         # table_approach 요청 (from_node 포함, 이어 하기 때 그대로 다시 보냄)
        self.quiet_obstacle = False   # leave quiet_to_next: 다음 노드 도착까지 LiDAR 장애물 판단 안 함 (좁은 틈 통과)
        self.skip_turn = False        # 다음 방향 바뀜 회전 생략 (축을 이미 맞춤)
        self.corridor = corridor      # 축 고정 후진을 docking(라이다 벽 따라가기)에 맡김
        self.handover = []            # corridor_approach로 맡긴 노드들 (지날 때마다 앞에서 빠짐)
        self.handover_req = None
        self.prec_info = {}           # docking 결과에 붙은 값 (corridor reached · then_done)
        self.prec_result = ''         # 마지막 정밀 정차 결과 detail
        self._last_pose = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------ 입력
    def on_move(self, cmd_id, append, drive, waypoints, guide=False, goal=''):
        if not waypoints:
            return
        if not guide:
            self.in_guide = False     # 안내가 끝났다 (복귀 · 서빙)
        if not append:  # REPLACE: 지금 가던 것까지 버리고 새로
            if self.phase == 'leave' and self.leave_req:
                self.leave_from = self.leave_req['node']   # 아직 못 떠났다 -> 새 명령 출발 때 다시
            self._stop_motion()
            self.queue.apply(cmd_id, False, drive, waypoints, guide, goal)
            self.handover, self.handover_req = [], None
            self.cur = None
            self.phase = 'idle'
            self.error = False
            return
        if not self.queue.apply(cmd_id, True, drive, waypoints, guide, goal):
            self.io.log(f'다른 명령({cmd_id})에 붙이는 APPEND 무시 (지금 {self.queue.cmd_id})')
            return
        if self.phase == 'corridor_approach' and self.prec_state == 'running' and self.handover_req:
            self._corridor_extend()
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
            self.in_guide = False
            self.handover, self.handover_req = [], None
            self.queue.clear()
            self.cur = None
            self.phase = 'idle'
            self.paused = False

    def on_nav_result(self, ok):
        if self.phase == 'nav' and self.nav_state == 'running':
            self.nav_state = 'ok' if ok else 'fail'

    def on_precision_result(self, ok, detail='', info=None):
        if self.phase in ('align', 'precision', 'leave', 'tape_approach', 'corridor_approach','table_approach') \
                and self.prec_state == 'running':
            self.prec_state = 'ok'
            self.prec_ok = ok
            self.prec_info = dict(info or {})
            self.prec_result = detail     # 통로: ok여도 'timeout...'이면 실패로 봄
            if not ok and self.phase == 'corridor_approach':
                self.io.log(f'통로 docking 실패: {detail}')
            elif not ok:
                self.io.log(f'정밀 정차 실패, Nav2 위치로 진행: {detail}')
                self.prec_detail = f'{self.cur.node_id if self.cur else ""}:{detail}'

    def on_precision_progress(self, node):
        """통로 docking이 노드를 지남 (가로선 · 거리) -> NODE_REACHED, 멈추지 않음. 마지막 노드는 결과로"""
        if self.phase != 'corridor_approach' or self.prec_state != 'running':
            return
        names = [s_.node_id for s_ in self.handover]
        if node not in names[:-1]:
            return
        while self.handover and self.handover[0].node_id != node:
            self._pass_step(self.handover.pop(0))
        self._pass_step(self.handover.pop(0))
        self.cur = self.handover[0]

    def _pass_step(self, st):
        self.io.event(NODE_REACHED, st.cmd_id, st.node_id, '')
        self.last_node = st.node_id
        self.last_drive = st.drive
        self.prev_xy = (st.x, st.y)

    def on_guide_result(self, ok, detail=''):
        if self.phase in ('guide_greet', 'guide_watch') and self.guide_state == 'running':
            self.guide_state = 'ok' if ok else 'fail'
            self.guide_detail = detail

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
        if pose is not None:
            self._last_pose = pose
        if pose is None or self.paused or self.error:
            return
        moving = self.phase in ('nav', 'reverse') and not self.quiet_obstacle   # 정밀 동작(tape 등) 중에는 장애물 판단 안 함
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
        elif self.phase in ('turn', 'final_turn', 'guide_turn'):
            w, done = turn_command(pose[2], self.turn_target, tol=self.yaw_tol)
            self.io.cmd_vel(0.0, w)
            if done:
                self.io.cmd_vel(0.0, 0.0)
                if self.phase == 'turn':
                    self._begin_move(pose)
                elif self.phase == 'guide_turn':
                    self.guide_after(pose)
                else:
                    self._precision_or_done()
        elif self.phase in ('guide_greet', 'guide_watch'):
            if self.guide_state == 'ok':
                self.guide_state = ''
                self.guide_after(pose)
            elif self.guide_state == 'fail':
                self._guest_lost()
        elif self.phase == 'nav':
            self._tick_nav(pose)
        elif self.phase == 'reverse':
            if self.axis is not None:
                oy = self._odom_yaw()
                err = wrap(self.axis_odom - oy) if (oy is not None and self.axis_odom is not None) else None
                v, w, done = self.rev.command_axis(pose, (self.cur.x, self.cur.y), self.axis, err)
            else:
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
                    if self.prec_ok and self.stops.get(self.cur.node_id, {}).get('hold_axis'):
                        self.axis = pose[2]             # 정렬한 방향 = 이어지는 후진의 축 (출발 회전 생략)
                        self.axis_odom = self._odom_yaw()
                        self.skip_turn = True
                    self._after_node(pose)
                else:
                    self._done()
        elif self.phase == 'leave':
            if self.prec_state == 'ok':
                self.prec_state = ''
                req, self.leave_req = self.leave_req or {}, None
                if self.prec_ok and req.get('quiet_to_next'):
                    self.quiet_obstacle = True
                if self.prec_ok and req.get('hold_axis'):
                    self.axis = pose[2]
                    self.axis_odom = self._odom_yaw()
                    self.skip_turn = True
                if self.prec_ok and req.get('to') == self.cur.node_id:
                    self.io.log(f'{req.get("node")} {req.get("method")} -> {self.cur.node_id} 도착')
                    self._reached(pose)
                else:
                    self._depart(pose)
        elif self.phase == 'tape_approach':
            if self.prec_state == 'ok':
                self.prec_state = ''
                self.prec_done_node = self.cur.node_id
                self._reached(pose)
        elif self.phase == 'table_approach':
            if self.prec_state == 'ok':
                self.prec_state = ''
                self.prec_done_node = self.cur.node_id   # 정렬 · 정지선까지 끝 = T 도착
                self.table_req = None
                self._reached(pose)
        elif self.phase == 'corridor_approach':
            if self.prec_state == 'ok':
                self.prec_state = ''
                if not self.prec_ok or str(self.prec_result).startswith('timeout'):
                    self._corridor_failed()
                    return
                steps, self.handover, req = self.handover, [], self.handover_req or {}
                self.handover_req = None
                names = [s_.node_id for s_ in steps]
                reached = self.prec_info.get('reached')
                idx = names.index(reached) if reached in names else len(steps) - 1
                for st in steps[:idx]:            # progress가 안 온 지나온 노드
                    self._pass_step(st)
                self.cur = steps[idx]
                if steps[idx + 1:]:               # 이어 붙인 게 늦어 docking이 먼저 섬 -> 큐로 돌려 여기서 다시
                    self.io.log(f'통로: {reached}에서 섬, {names[idx + 1:]}는 다시 넘겨받기')
                    self.queue.steps[0:0] = steps[idx + 1:]
                then_done = self.prec_info.get('then_done', bool(req.get('then')) and idx == len(steps) - 1)
                if then_done:
                    self.prec_done_node = self.cur.node_id   # tape / dock_in을 이미 함
                self._reached(pose)
        elif self.phase == 'grace':
            if len(self.queue):
                self._start_next(pose)
            elif now >= self.grace_until:
                self._finish(pose)

    # ------------------------------------------------------------ 내부
    
    def _tick_nav(self, pose):
        d = math.hypot(self.cur.x - pose[0], self.cur.y - pose[1])
        nxt = self.queue.steps[0] if len(self.queue) else None
        if nxt and nxt.drive == FORWARD and d < self.pass_tol and not self._is_check(self.cur, nxt) and not self._table_next(nxt):
            self._reached(pose)  # 중간 노드: 위치만 통과, 다음 목표로 바로 (손님 확인 노드는 끝까지 가서 선다)
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
        if self.prev_xy is None:
            self.prev_xy = (pose[0], pose[1])
        if self.cur.guide and not self.in_guide:
            # 안내 첫 출발: 떠나기 전에 앞(카메라)으로 손님을 기다린다
            self.in_guide = True
            self.check_xy = (pose[0], pose[1])
            self._guide_request('greet', self.last_node or self.cur.node_id, self._depart)
            return
        self._depart(pose)

    

    def _depart(self, pose):
        leave = self.stops.get(self.leave_from, {}).get('leave') if self.leave_from else None
        if leave:  # 이 노드를 떠날 때 할 정밀 동작 (언도킹 · 테이프 빼기)
            self.leave_req = dict(leave, node=self.leave_from)
            self.leave_from = ''
            self.phase = 'leave'
            self.prec_state = 'running'
            self.io.direction('STOP')
            self.io.precision(self.leave_req)
            return
        if self.last_drive is not None and self.cur.drive != self.last_drive and self.skip_turn:
            self.skip_turn = False    # 마커로 축을 이미 맞춤 -> 돌지 않고 바로
            self._begin_move(pose)
            return
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
            self.axis = None
            self.axis_odom = None
            self.skip_turn = False
            # ↓↓↓ 추가: 테이블 도착지면 Nav2로 T 좌표까지 가지 않고, 지금 자리(직전 노드)에서 docking에 맡김
            stop = self.stops.get(self.cur.node_id, {})
            if (stop.get('method') == 'table' and self._precise_here(self.cur)
                    and self.last_node in stop.get('from', {})):
                self.io.cmd_vel(0.0, 0.0)   # 여기서부터 /cmd_vel은 docking(table)이 씀
                self.table_req = dict(stop, node=self.cur.node_id, from_node=self.last_node)
                self.phase = 'table_approach'
                self.prec_state = 'running'
                self.io.direction('STOP')   # 테이블 옆이라 LiDAR 장애물 판단 끔
                self.io.precision(dict(self.table_req))
                return
            # ↑↑↑ 추가 끝
            self.phase = 'nav'  # nav_go 결과가 바로 와도 받을 수 있게 먼저
            self.nav_state = 'running'
            self.nav_tries = 0
            self.io.direction('FORWARD')
            self.io.nav_go(self.cur.x, self.cur.y, self.cur.yaw)
        else:
            if self.axis is not None and self.corridor and self._corridor_handover(pose):
                return
            stop = self.stops.get(self.cur.node_id, {})
            if (self.axis is not None and stop.get('method') == 'tape' and stop.get('approach', True)
                    and self._precise_here(self.cur)):
                # 축 고정 후진으로 테이프 노드: 좌표 대신 바닥 가로선으로 멈추게 tape에 바로 맡김
                self.io.cmd_vel(0.0, 0.0)  # 여기서부터 /cmd_vel은 docking(tape)이 씀
                self.phase = 'tape_approach'
                self.prec_state = 'running'
                self.io.direction('STOP')   # 픽업대 · 벽에 붙어야 하니 LiDAR 장애물 판단 끔
                self.io.precision(dict(stop, node=self.cur.node_id, approach=True))
                return
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
        elif self.phase == 'tape_approach':
            self.prec_state = 'running'
            self.io.precision(dict(self.stops.get(self.cur.node_id, {}), node=self.cur.node_id, approach=True))
        elif self.phase == 'table_approach' and self.table_req:   # ← 추가 (테이블)
            self.prec_state = 'running'
            self.io.precision(dict(self.table_req))    
        elif self.phase == 'corridor_approach' and self.handover_req:
            # 멈췄던 자리에서 남은 노드로 다시 맡김 (첫 선은 가까워도 셈)
            self.prec_state = 'running'
            self.handover_req.update(nodes=[s_.node_id for s_ in self.handover], from_node=self.last_node, resume=True)
            self.io.precision(dict(self.handover_req))
        elif self.phase == 'leave' and self.leave_req:
            self.prec_state = 'running'
            self.io.precision(self.leave_req)
        elif self.phase in ('guide_greet', 'guide_watch'):
            self.guide_state = 'running'
            self.io.guide({'action': 'greet' if self.phase == 'guide_greet' else 'watch',
                           'node': self.cur.node_id if self.cur else self.last_node})

    def _reached(self, pose):
        step = self.cur
        self.quiet_obstacle = False
        self.last_node = step.node_id
        self.last_drive = step.drive
        self.nav_state = ''
        self.io.event(NODE_REACHED, step.cmd_id, step.node_id, '')
        nxt = self.queue.steps[0] if len(self.queue) else None
        check = self._is_check(step, nxt)
        self.prev_xy = (step.x, step.y)
        if check:
            self._begin_check(pose)
            return
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
        stop = self.stops.get(self.cur.node_id, {})
        keep_axis = self.axis is not None or stop.get('no_final_turn')   # 축 그대로 (테이프가 이어서 맞춤)
        if self.cur.drive == REVERSE and not keep_axis and abs(wrap(self.cur.yaw - pose[2])) > self.yaw_tol:
            self.turn_target = self.cur.yaw  # FORWARD는 Nav2가 이미 맞춤
            self.phase = 'final_turn'
            return
        self._precision_or_done()

    def _precision_or_done(self):
        if self.prec_done_node and self.prec_done_node == self.cur.node_id:
            self.prec_done_node = ''      # tape_approach에서 이미 함
            self._done()
            return
        stop = self.stops.get(self.cur.node_id)
        if stop and stop['method'] in ('dock', 'tape') and self._precise_here(self.cur):
            self.phase = 'precision'
            self._request_precision(False)
            return
        self._done()

    def _at_goal(self, step):
        """이 노드가 이번 이동의 도착지인가 (Move.goal_node). 모르면(예전 서버) 줄 끝(뒤에 남은 노드 없음)이면 도착지"""
        if step.goal:
            return step.node_id == step.goal
        return len(self.queue) == 0

    def _precise_here(self, step):
        """정밀 정차(dock · tape)를 할 곳인가: 도착지이고 안내가 아닐 때 (안내는 Nav2 위치 · yaw로만 섬)"""
        return self._at_goal(step) and not step.guide

    def _table_next(self, nxt):
        """다음 step이 정밀 정차할 테이블(도착지)인가 -> 그 앞 노드는 통과하지 말고 끝까지 가서 선다"""
        return (nxt is not None and nxt.drive == FORWARD and not nxt.guide
                and self.stops.get(nxt.node_id, {}).get('method') == 'table'
                and nxt.goal == nxt.node_id)

    def _request_precision(self, align):
        stop = dict(self.stops.get(self.cur.node_id, {}))
        stop['node'] = self.cur.node_id
        self.prec_state = 'running'
        self.io.precision(stop)

    def _done(self):
        # 정밀 정차를 못 했으면 이유를 reason에 (서버가 경고만 띄우고 작업은 이어 감)
        self.io.event(CMD_DONE, self.queue.cmd_id, self.cur.node_id, self.prec_detail)
        self.prec_detail = ''
        if self.stops.get(self.cur.node_id, {}).get('leave') and self._precise_here(self.cur):
            self.leave_from = self.cur.node_id   # 다음 출발 때 leave 먼저 (정밀 정차한 도착지에서만)
        self.phase = 'idle'
        self.io.direction('STOP')
        if self.cur.guide and self._at_goal(self.cur):
            self.io.guide({'action': 'arrive', 'node': self.cur.node_id})  # 도착 효과 (기다리지 않음)

    def _travel_to(self, pose, step):
        """축 따라 뒤로 (pose -> step 노드) 남은 거리 (지도 좌표, 옆으로 비킨 도크는 그 줄까지)"""
        b = self.axis + math.pi
        return max(0.0, (step.x - pose[0]) * math.cos(b) + (step.y - pose[1]) * math.sin(b))

    def _corridor_plan(self, run, rest):
        """run(이어지는 REVERSE 노드들) 안 도착지 -> (끝 index, then) / 통로용이 아닌 정밀 정차면 (None, None)
        rest = run 뒤에 큐에 더 남은 노드 수 (goal_node가 없을 때 줄 끝 판단)"""
        for i, st in enumerate(run):
            last = i == len(run) - 1 and rest == 0
            at_goal = (st.node_id == st.goal) if st.goal else last
            if at_goal and not st.guide:
                stop = self.stops.get(st.node_id, {})
                if stop.get('method') in ('tape', 'dock_in') and stop.get('corridor', True):
                    return i, dict(stop, node=st.node_id)
                if stop.get('method'):
                    return None, None     # 도착지 정밀 정차가 통로용이 아님 -> 예전 방식
                return i, None
        return len(run) - 1, None

    def _corridor_handover(self, pose):
        """축 고정 후진을 docking에 맡긴다 (method=corridor) -> 맡겼으면 True"""
        run = [self.cur]
        for st in self.queue.steps:
            if st.drive != REVERSE:
                break
            run.append(st)
        target, then = self._corridor_plan(run, len(self.queue) - (len(run) - 1))
        if target is None:
            return False
        req = {'method': 'corridor', 'node': run[target].node_id, 'nodes': [s_.node_id for s_ in run[:target + 1]],
               'then': then, 'from_node': self.last_node,
               'timeout': max(90.0, float((then or {}).get('timeout', 0.0)) + 40.0)}
        self.handover = run[:target + 1]
        del self.queue.steps[:target]     # cur는 이미 꺼냄. 뒤의 target개를 맡김
        self.handover_req = req
        self.io.cmd_vel(0.0, 0.0)
        self.phase = 'corridor_approach'
        self.prec_state = 'running'
        self.io.direction('STOP')         # 통로는 좁고 벽에 붙음: LiDAR 장애물 판단 끔 (tape_approach와 같음)
        self.io.log(f'통로 넘겨받기: {req["nodes"]} then {(then or {}).get("method")}')
        self.io.precision(dict(req))
        return True

    def _corridor_failed(self):
        """통로 docking 실패 (시간 초과 포함): 로봇은 통로 어딘가에 서 있다 -> 도착으로 치지 않는다
        progress로 이미 보낸 노드까지만 지난 것 (result.reached가 그보다 앞이면 그 앞 노드까지 보냄)
        남은 노드는 handover에 그대로 두고 CMD_FAILED reason="corridor_failed:<맡긴 끝 노드>:<이유>"
        -> 서버는 고장 처리 (lock 유지, 직원 확인 후 재개). 재개(RESUME)하면 _resume이 남은 노드로 다시 맡김"""
        names = [s_.node_id for s_ in self.handover]
        reached = self.prec_info.get('reached')
        if reached in names[:-1]:
            idx = names.index(reached)
            for st in self.handover[:idx]:
                self._pass_step(st)
            self.handover = self.handover[idx:]
        if self.handover:
            self.cur = self.handover[0]
        end = self.handover[-1].node_id if self.handover else self.last_node
        reason = f'corridor_failed:{end}:{self.prec_result or "fail"}'
        self.prec_detail = ''
        self.io.cmd_vel(0.0, 0.0)
        self.io.log(f'통로 실패 -> 멈춤 ({reason})')
        self.io.event(CMD_FAILED, self.queue.cmd_id, self.last_node, reason)
        self.error = True             # phase · handover · handover_req 유지 (RESUME이면 이어서)

    def _corridor_extend(self):
        """통로 후진 중 APPEND: 이어지는 REVERSE 노드를 달리는 docking에 붙임 (then이 이미 있으면 안 붙임)"""
        if self.handover_req.get('then'):
            return
        run = list(self.handover)
        for st in self.queue.steps:
            if st.drive != REVERSE:
                break
            run.append(st)
        if len(run) == len(self.handover):
            return
        target, then = self._corridor_plan(run, len(self.queue) - (len(run) - len(self.handover)))
        if target is None or target < len(self.handover):
            return
        add = target + 1 - len(self.handover)
        added = [s_.node_id for s_ in run[len(self.handover):target + 1]]
        self.handover = run[:target + 1]
        del self.queue.steps[:add]
        # nodes = 이 요청의 처음부터 전체 (docking은 지난 노드 뒤에 새 노드만 붙임)
        self.handover_req.update(node=self.handover[-1].node_id, nodes=list(self.handover_req['nodes']) + added,
                                 then=then)
        if then:
            self.handover_req['timeout'] = max(float(self.handover_req.get('timeout', 90.0)),
                                               float(then.get('timeout', 0.0)) + 40.0)
        f = getattr(self.io, 'precision_extend', None)
        if f:
            f({'nodes': self.handover_req['nodes'], 'then': then})
        self.io.log(f'통로 이어 감: {self.handover_req["nodes"]} then {(then or {}).get("method")}')

    def _odom_yaw(self):
        f = getattr(self.io, 'odom_yaw', None)
        try:
            return f() if f else None
        except Exception:
            return None

    # ------------------------------------------------------------ 안내
    def _is_check(self, step, nxt):
        """안내 중 꺾이는 노드인가 (들어온 방향 vs 다음 노드로 나갈 방향)"""
        if not (self.in_guide and step.guide and nxt and nxt.guide) or REVERSE in (step.drive, nxt.drive):
            return False
        if self.prev_xy is None:
            return False
        if self.check_xy and math.hypot(step.x - self.check_xy[0], step.y - self.check_xy[1]) < self.check_gap:
            return False          # 방금 인사 · 확인한 자리 근처
        h_in = math.atan2(step.y - self.prev_xy[1], step.x - self.prev_xy[0])
        h_out = math.atan2(nxt.y - step.y, nxt.x - step.x)
        return abs(wrap(h_out - h_in)) > self.check_turn

    def _guide_request(self, action, node, after):
        self.io.cmd_vel(0.0, 0.0)
        self.io.nav_cancel()
        self.io.direction('STOP')
        self.phase = 'guide_greet' if action == 'greet' else 'guide_watch'
        self.guide_state = 'running'
        self.guide_after = after
        self.io.guide({'action': action, 'node': node})

    def _begin_check(self, pose):
        """손님 확인 노드: 멈춤 -> 뒤로 돎 -> watch -> 다음 노드 쪽으로 돎 -> 이어 감"""
        self.io.cmd_vel(0.0, 0.0)
        self.io.nav_cancel()
        self.io.direction('STOP')
        node = self.cur.node_id
        self.check_xy = (self.cur.x, self.cur.y)
        self.phase = 'guide_turn'
        self.turn_target = wrap(pose[2] + math.pi)
        self.io.log(f'{node}: 손님 확인 (뒤로 돎)')

        def after_back(p):
            self._guide_request('watch', node, after_watch)

        def after_watch(p):
            nxt = self.queue.steps[0] if len(self.queue) else None
            if nxt is None:  # 확인 노드가 줄 끝: 그대로 끝 처리
                self._after_node(p)
                return
            heading = math.atan2(nxt.y - p[1], nxt.x - p[0])
            self.phase = 'guide_turn'
            self.turn_target = wrap(heading + math.pi) if nxt.drive == REVERSE else heading
            self.guide_after = self._after_node

        self.guide_after = after_back

    def _guest_lost(self):
        node = self.cur.node_id if self.cur else self.last_node
        reason = f'guest_lost:{node}:{self.guide_detail or "not_seen"}'
        self.io.log(f'손님을 놓침 -> 멈춤 ({reason})')
        self.io.event(CMD_FAILED, self.queue.cmd_id, node, reason)
        self.queue.clear()
        self.cur = None
        self.in_guide = False
        self.guide_state = ''
        self.phase = 'idle'
        self.io.direction('STOP')

    def _stop_motion(self, keep=False, keep_dir=False):
        self.io.cmd_vel(0.0, 0.0)
        self.io.nav_cancel()
        if self.phase in ('align', 'precision', 'leave', 'tape_approach', 'corridor_approach','table_approach'):
            self.io.precision_cancel()
        if self.phase in ('guide_greet', 'guide_watch'):
            self.io.guide_cancel()
        if not keep:
            self.nav_state = ''
            self.prec_state = ''
            self.holding = False
            self.policy.update(0.0, '')
            self.io.lcd_override(None)
        if not keep_dir:
            self.io.direction('STOP')

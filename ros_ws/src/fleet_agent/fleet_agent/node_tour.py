"""노드 순회 시험: 서버 없이 로봇 혼자 node_graph의 노드들을 Nav2로 차례로 지나가 본다.

fleet_agent에 /fleet/move를 직접 보내고 /fleet/event를 받아서, 노드마다 걸린 시간과
도착했을 때 실제 위치(TF)와 노드 좌표의 차이를 찍는다. 서버(main_service)는 끄고 쓴다.

  ros2 run fleet_agent node_tour --preset hall      # D1 빠지기(교점에서 360° 회전) -> 홀 N1~N13 + 테이블 보기 (N3·N6·N9는 양쪽)
  ros2 run fleet_agent node_tour                    # 기본 코스: 홀 N1~N13 지그재그 후 N1로 (N1에서 시작)
  ros2 run fleet_agent node_tour N1 N4 N13 N11 N1   # 지나갈 노드를 직접 (사이 노드는 길 찾기로 채움)
  ros2 run fleet_agent node_tour --step 2 N1 N4 N1  # 목표 노드마다 멈춰 2초 쉬고 다음으로
  ros2 run fleet_agent node_tour --loops 3          # 같은 코스 3번
  ros2 run fleet_agent node_tour --tables           # 테이블 앞 노드(N2~N10)마다 서서 가장 가까운 테이블 모서리를 봄
  ros2 run fleet_agent node_tour --tables all       # 그 노드가 맡은 테이블 모두 (N3 = T2, T1 차례로)
  ros2 run fleet_agent node_tour --start D1 --spin 360 N1 N2 ...   # 도크에서 시작: leave(빠지기) -> 첫 노드 -> 한 바퀴

  --preset hall = --start D1 --tables all --keep-going N1 N2 N3 N4 N7 N6 N5 N8 N11 N12 N9 N10 N13
     + --spin 360: 첫 노드(N1)에서 한 바퀴 (AMCL 맞춤. 시스템은 시작 전 위치 맞추기 LOCALIZE로 한 번 함)
     로봇 쪽: robot_system start_node:=D1 use_docking:=true precision_stops:=.../precision_stops.yaml (spin_deg 0)
  --start <도크>: 첫 Move에 도크를 넣지 않는다 (fleet_agent가 start_node의 leave = 빠지기를 먼저 하고 첫 노드로 Nav2)
  --spin <deg>: (따로 시험할 때) 첫 노드에 서면 제자리 회전 (cmd_vel, odom) -> AMCL 맞춤. 앞뒤 AMCL 불확실성을 기록
  --tables: 테이블 모서리 = node_graph T 좌표 (fleet_agent가 테이블 도착지일 때 직전 노드에서 보는 점, from = precision_stops).
            그 노드에서 다리를 끊고 마지막 yaw = 모서리 쪽으로 보내서, 도착 뒤 방향 오차를 잰다

  - 후진 구간(node_graph reverse)과 도킹 노드는 목표로 적지 않으면 지나가지 않는다 (전진 주행만 시험)
  - 지나가는 노드의 yaw = 다음 노드 쪽, 마지막 노드 = node_graph에 yaw가 있으면 그 값, 없으면 들어온 방향

로그 (~/tour_logs/tour_<시각>_*): 문제 찾기용
  .log        보낸 Move(좌표 · yaw), 받은 이벤트 전부(reason 포함), 노드마다 위치 · 앞뒤/옆 오차 · AMCL, 회전 앞뒤 AMCL
              + /rosout: fleet_agent · docking · obstacle INFO 이상, Nav2(bt_navigator · controller 등) WARN 이상 (<노드 I/W/E>)
  _trace.csv  0.2초마다 t, x, y, yaw(map), amcl_std, cmd v, w, 지금 다리 · 목표 노드 (지나간 길 그림용)
  _result.csv 결과 표
"""
import argparse
import csv
import math
import os
import sys
import time
from collections import deque

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from fleet_msgs.msg import Control, Move, RobotEvent, Waypoint
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener

DEFAULT_ROUTE = ['N1', 'N2', 'N3', 'N4', 'N7', 'N6', 'N5', 'N8', 'N9', 'N10',
                 'N13', 'N12', 'N11', 'N8', 'N5', 'N2', 'N1']
PRESETS = {
    # D1 빠지기 -> N1에서 360° (AMCL) -> 홀 13개 노드를 한 번씩
    # (윗줄 -> 2줄 -> N8 -> 아랫줄 -> 3줄 -> N13), 테이블 앞 노드에서 모서리 보기 (N3 · N6 · N9는 양쪽 테이블 둘 다)
    'hall': dict(start='D1', spin=360.0, tables='all',
                 route=['N1', 'N2', 'N3', 'N4', 'N7', 'N6', 'N5', 'N8', 'N11', 'N12', 'N9', 'N10', 'N13']),
}
EV_NAME = {0: 'NODE_REACHED', 1: 'CMD_DONE', 2: 'CMD_FAILED', 3: 'STATE', 4: 'BLOCKED', 5: 'DOCKED', 6: 'UNDOCKED',
           7: 'LOCALIZED'}
SPIN_W = 0.5            # rad/s 제자리 회전 속도
ROSOUT_FLEET = {'fleet_agent', 'docking', 'obstacle', 'initial_pose'}          # INFO 이상을 로그에
ROSOUT_NAV2 = {'bt_navigator', 'controller_server', 'planner_server', 'behavior_server',
               'amcl', 'velocity_smoother', 'smoother_server'}                 # WARN 이상만
SPIN_TIMEOUT = 40.0     # s


class Graph:
    def __init__(self, path):
        with open(path, encoding='utf-8') as f:
            g = yaml.safe_load(f)
        self.nodes = {n['id']: n for n in g['nodes']}
        self.rev = set()
        for chain in g.get('reverse') or []:
            for a, b in zip(chain, chain[1:]):
                self.rev.add((a, b))

    # 너비 우선 탐색으로 가장 적은 칸 수의 길 (후진 길 · 도킹 · 픽업은 목적지가 아니면 안 지나감)
    def path(self, a, b):
        if a == b:
            return [a]
        prev = {a: None}
        q = deque([a])
        while q:
            u = q.popleft()
            for v in self.nodes[u].get('next', []):
                if v in prev or (u, v) in self.rev:
                    continue
                if v != b and self.nodes[v].get('stop_kind') in ('DOCK', 'PICKUP', 'QUEUE'):
                    continue
                prev[v] = u
                if v == b:
                    out = [b]
                    while prev[out[-1]] is not None:
                        out.append(prev[out[-1]])
                    return out[::-1]
                q.append(v)
        return None


def heading(n1, n2):
    return math.atan2(n2['y'] - n1['y'], n2['x'] - n1['x'])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Tour(Node):
    def __init__(self, graph, legs, step_sec, faces=None, spin_after=None, spin_deg=0.0, log_base=''):
        super().__init__('node_tour')
        self.g = graph
        self.legs = legs          # [[노드, ...], ...] 한 다리 = Move 하나
        self.step_sec = step_sec
        self.faces = faces or {}  # {다리 index: (테이블, yaw)} 다리 끝에서 테이블 모서리를 봄
        self.yaw_end = {}         # {다리 index: yaw} 다리 끝 yaw 강제 (도크에서 나온 첫 노드 = 다음 노드 쪽)
        self.spin_after = spin_after   # 이 다리가 끝나면 제자리 회전
        self.spin_deg = spin_deg
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Move, '/fleet/move', rel)
        self.ctl = self.create_publisher(Control, '/fleet/control', rel)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(RobotEvent, '/fleet/event', self.on_event, rel)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, amcl_qos)
        self.create_subscription(Log, '/rosout', self.on_rosout, QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.cmd_id = int(time.time()) % 1000000 * 100 + 50
        self.my_ids = set()
        self.leg = -1
        self.t_leg = self.t_node = 0.0
        self.t0 = time.monotonic()
        self.rows = []            # (노드, 걸린 초, 위치 오차 m, 방향 오차 deg, 앞뒤 m, 옆 m)
        self.failed = None
        self.fails = []           # 실패한 다리 (keep_going이면 넘어감)
        self.fail_run = 0
        self.keep_going = False
        self.done = False
        self.wait_until = None
        self.prev_node = None
        self.amcl_std = None
        self.amcl_yaw_std = None
        self.cmd = (0.0, 0.0)
        self.spin = None          # 회전 중: {'last': yaw, 'turned': rad, 't': 시작}
        self.target = ''
        # 로그 파일
        self.log_f = self.trace_w = self.trace_f = None
        self.log_base = log_base
        if log_base:
            os.makedirs(os.path.dirname(log_base), exist_ok=True)
            self.log_f = open(log_base + '.log', 'w', encoding='utf-8')
            self.trace_f = open(log_base + '_trace.csv', 'w', newline='', encoding='utf-8')
            self.trace_w = csv.writer(self.trace_f)
            self.trace_w.writerow(['t', 'x', 'y', 'yaw', 'amcl_std', 'amcl_yaw_std_deg', 'v', 'w', 'leg', 'target'])
        self.create_timer(0.2, self.on_trace)
        self.create_timer(0.1, self.on_spin_timer)

    # ------------------------------------------------------------ 로그
    def log(self, text, level='info'):
        t = time.monotonic() - self.t0
        if self.log_f:                 # 파일 먼저 (화면 로그가 실패해도 남게)
            self.log_f.write(f'[{t:7.2f}] {text}\n')
            self.log_f.flush()
        lg = self.get_logger()         # rclpy: 한 줄(호출 자리)에서 심각도를 바꾸면 ValueError -> 심각도마다 따로 부름
        if level == 'error':
            lg.error(text)
        elif level == 'warn':
            lg.warning(text)
        else:
            lg.info(text)

    def on_rosout(self, m):
        """fleet_agent · docking · obstacle 로그(INFO 이상)와 Nav2 경고 · 에러를 투어 로그에 같이 (실패 원인 찾기)"""
        name = m.name.split('.')[-1]
        if name == 'node_tour' or not self.log_f:
            return
        if name in ROSOUT_FLEET and m.level >= 20 or name in ROSOUT_NAV2 and m.level >= 30:
            lv = {10: 'D', 20: 'I', 30: 'W', 40: 'E', 50: 'F'}.get(m.level, str(m.level))
            self.log_f.write(f'[{time.monotonic() - self.t0:7.2f}]     <{name} {lv}> {m.msg}\n')
            self.log_f.flush()

    def on_amcl(self, msg):
        c = msg.pose.covariance
        self.amcl_std = math.sqrt(max(c[0], 0.0) + max(c[7], 0.0))
        self.amcl_yaw_std = math.degrees(math.sqrt(max(c[35], 0.0)))

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def amcl_text(self):
        if self.amcl_std is None:
            return 'AMCL -'
        return f'AMCL {self.amcl_std * 100:.1f}cm/{self.amcl_yaw_std:.1f}°'

    def on_spin_timer(self):
        if self.spin is not None:
            self.tick_spin()

    def on_trace(self):
        if not self.trace_w:
            return
        p = self.pose()
        if p is None:
            return
        self.trace_w.writerow([f'{time.monotonic() - self.t0:.2f}', f'{p[0]:.4f}', f'{p[1]:.4f}', f'{p[2]:.4f}',
                               '' if self.amcl_std is None else f'{self.amcl_std:.4f}',
                               '' if self.amcl_yaw_std is None else f'{self.amcl_yaw_std:.2f}',
                               f'{self.cmd[0]:.3f}', f'{self.cmd[1]:.3f}', self.leg, self.target])

    # ------------------------------------------------------------ 이동
    def send_leg(self):
        self.leg += 1
        if self.leg >= len(self.legs):
            self.done = True
            return
        nodes = self.legs[self.leg]
        m = Move()
        self.cmd_id += 1
        self.my_ids.add(self.cmd_id)
        m.cmd_id = self.cmd_id
        m.mode = Move.REPLACE
        m.drive = Move.FORWARD
        m.goal_node = nodes[-1]           # 이 다리의 끝 = 도착지 (정밀 정차 노드여도 거기서만)
        for i, nid in enumerate(nodes):
            n = self.g.nodes[nid]
            w = Waypoint()
            w.node_id, w.x, w.y = nid, float(n['x']), float(n['y'])
            if i + 1 < len(nodes):
                w.yaw = heading(n, self.g.nodes[nodes[i + 1]])
            elif self.leg in self.faces:
                w.yaw = float(self.faces[self.leg][1])
            elif self.leg in self.yaw_end:
                w.yaw = float(self.yaw_end[self.leg])
            elif 'yaw' in n:
                w.yaw = float(n['yaw'])
            elif i > 0:
                w.yaw = heading(self.g.nodes[nodes[i - 1]], n)
            m.waypoints.append(w)
        self.t_leg = self.t_node = time.monotonic()
        self.target = nodes[-1]
        self.pub.publish(m)
        wps = ' '.join(f'{w.node_id}({w.x:.3f},{w.y:.3f},{math.degrees(w.yaw):+.0f}°)' for w in m.waypoints)
        face = f'  [끝에서 {self.faces[self.leg][0]} 모서리 보기]' if self.leg in self.faces else ''
        self.log(f'[{self.leg + 1}/{len(self.legs)}] Move {m.cmd_id}: {wps}{face}')

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def odom_yaw(self):
        try:
            t = self.tf.lookup_transform('odom', 'base_footprint', Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    # ------------------------------------------------------------ 제자리 회전 (AMCL 맞춤)
    def start_spin(self):
        y = self.odom_yaw()
        if y is None:
            self.log('회전: odom TF 없음 -> 건너뜀', 'warn')
            self.after_spin()
            return
        p = self.pose()
        self.log(f'회전 시작 {self.spin_deg:.0f}°: 위치 {self.ptext(p)}, {self.amcl_text()}')
        self.spin = {'last': y, 'turned': 0.0, 't': time.monotonic(), 'p': p}
        self.target = 'spin'

    def tick_spin(self):
        y = self.odom_yaw()
        if y is not None:
            self.spin['turned'] += abs(wrap(y - self.spin['last']))
            self.spin['last'] = y
        goal = math.radians(self.spin_deg)
        left = goal - self.spin['turned']
        timeout = time.monotonic() - self.spin['t'] > SPIN_TIMEOUT
        tw = Twist()
        if left > 0.02 and not timeout:
            tw.angular.z = SPIN_W if left > 0.3 else max(0.15, SPIN_W * left / 0.3)
            self.cmd_pub.publish(tw)
            return
        self.cmd_pub.publish(tw)
        p0, p = self.spin['p'], self.pose()
        jump = '' if (p0 is None or p is None) else f', 회전 중 AMCL 위치 이동 {math.hypot(p[0] - p0[0], p[1] - p0[1]) * 100:.1f}cm'
        self.log(f'회전 끝: {math.degrees(self.spin["turned"]):.0f}° ({time.monotonic() - self.spin["t"]:.1f}초)'
                 f'{" 시간 초과" if timeout else ""}, 위치 {self.ptext(p)}, {self.amcl_text()}{jump}')
        self.spin = None
        self.wait_until = time.monotonic() + 1.0     # AMCL이 한 번 더 고칠 시간 -> tick()이 다음 다리

    def after_spin(self):
        self.send_leg()

    @staticmethod
    def ptext(p):
        return '-' if p is None else f'({p[0]:.3f}, {p[1]:.3f}, {math.degrees(p[2]):+.1f}°)'

    # ------------------------------------------------------------ 이벤트
    def on_event(self, e):
        name = EV_NAME.get(e.type, str(e.type))
        mine = e.cmd_id in self.my_ids
        reason = f' reason="{e.reason}"' if e.reason else ''
        if not mine:
            self.log(f'  (다른 명령 이벤트) {name} {e.node_id} cmd {e.cmd_id}{reason}')
            return
        if e.cmd_id != self.cmd_id:   # 버린(실패한) 다리에서 늦게 온 이벤트: 기록만
            self.log(f'  (지난 다리 이벤트) {name} {e.node_id} cmd {e.cmd_id}{reason}')
            return
        now = time.monotonic()
        p = self.pose()
        if e.type == RobotEvent.NODE_REACHED:
            n = self.g.nodes.get(e.node_id)
            err = dyaw = along = lat = float('nan')
            if n and p:
                err = math.hypot(p[0] - n['x'], p[1] - n['y'])
                if self.prev_node and self.prev_node in self.g.nodes and self.prev_node != e.node_id:
                    h = heading(self.g.nodes[self.prev_node], n)
                    dx, dy = p[0] - n['x'], p[1] - n['y']
                    along = dx * math.cos(h) + dy * math.sin(h)        # + = 노드를 지나침, - = 덜 옴
                    lat = -dx * math.sin(h) + dy * math.cos(h)         # + = 진행 방향 왼쪽
                w = next((w for w in self.last_waypoints() if w[0] == e.node_id), None)
                if w:
                    dyaw = math.degrees(abs(wrap(p[2] - w[1])))
            self.rows.append((e.node_id, now - self.t_node, err, dyaw, along, lat))
            al = '' if math.isnan(along) else f' (앞뒤 {along * 100:+.1f}, 옆 {lat * 100:+.1f})'
            self.log(f'  도착 {e.node_id:4s} {now - self.t_node:5.1f}초  위치 오차 {err * 100:5.1f}cm{al}  '
                     f'위치 {self.ptext(p)}  {self.amcl_text()}')
            self.t_node = now
            self.prev_node = e.node_id
        elif e.type == RobotEvent.CMD_DONE:
            self.fail_run = 0
            self.log(f'  다리 끝 {e.node_id} ({now - self.t_leg:.1f}초){reason}  위치 {self.ptext(p)}  {self.amcl_text()}')
            if self.leg in self.faces:
                table, want = self.faces[self.leg]
                dyaw = float('nan') if p is None else math.degrees(abs(wrap(p[2] - want)))
                self.rows.append((f'{e.node_id}>{table}', now - self.t_leg, float('nan'), dyaw,
                                  float('nan'), float('nan')))
                self.log(f'  {e.node_id} -> {table} 모서리 보기: 방향 오차 {dyaw:.1f}° '
                         f'(원하는 {math.degrees(want):+.1f}°, 지금 {math.degrees(p[2]) if p else float("nan"):+.1f}°)')
            if self.spin_after is not None and self.leg == self.spin_after and self.spin_deg > 0:
                self.start_spin()
            elif self.step_sec > 0 and self.leg + 1 < len(self.legs):
                self.wait_until = now + self.step_sec
            else:
                self.send_leg()
        elif e.type in (RobotEvent.DOCKED, RobotEvent.UNDOCKED):
            self.log(f'  {name} {e.node_id}{reason}  위치 {self.ptext(p)}  {self.amcl_text()}'
                     + ('  (실패: reason 확인)' if e.reason else ''))
        elif e.type in (RobotEvent.CMD_FAILED, RobotEvent.BLOCKED):
            what = f'{name} {e.node_id} {e.reason}'
            self.fails.append(f'[{self.leg + 1}] {what}')
            self.rows.append((f'!{e.node_id}', now - self.t_leg, float('nan'), float('nan'), float('nan'), float('nan')))
            self.fail_run += 1
            if self.keep_going and self.fail_run < 3 and self.leg + 1 < len(self.legs):
                # 이 다리는 버리고 다음 다리로 (투어 한 번에 노드 전부 데이터를 얻으려고). 연속 3번이면 멈춤
                self.log(f'  {what} -> 이 다리 버리고 1.5초 뒤 다음 다리  위치 {self.ptext(p)}  {self.amcl_text()}', 'error')
                stop = Control()
                stop.command = Control.CANCEL
                self.ctl.publish(stop)
                self.wait_until = now + 1.5
                return
            self.failed = what
            self.log(f'  {what} -> 순회 멈춤  위치 {self.ptext(p)}  {self.amcl_text()}', 'error')
            self.done = True
        else:
            self.log(f'  {name} {e.node_id}{reason}')

    def last_waypoints(self):
        nodes = self.legs[self.leg]
        out = []
        for i, nid in enumerate(nodes):
            n = self.g.nodes[nid]
            if i + 1 < len(nodes):
                out.append((nid, heading(n, self.g.nodes[nodes[i + 1]])))
        return out

    def tick(self):
        if self.wait_until and time.monotonic() >= self.wait_until:
            self.wait_until = None
            self.send_leg()

    def report(self):
        out = ['', '==== 노드 순회 결과 ====',
               f'{"노드":7s}{"초":>7s}{"위치오차cm":>12s}{"앞뒤cm":>9s}{"옆cm":>8s}{"방향오차deg":>13s}']
        for nid, sec, err, dyaw, along, lat in self.rows:
            d = '' if math.isnan(dyaw) else f'{dyaw:.0f}'
            e = '-' if math.isnan(err) else f'{err * 100:.1f}'
            a = '' if math.isnan(along) else f'{along * 100:+.1f}'
            s = '' if math.isnan(lat) else f'{lat * 100:+.1f}'
            out.append(f'{nid:7s}{sec:7.1f}{e:>12s}{a:>9s}{s:>8s}{d:>13s}')
        errs = [r[2] for r in self.rows if not math.isnan(r[2])]
        if errs:
            out.append(f'도착 {len(errs)}개, 평균 오차 {sum(errs) / len(errs) * 100:.1f}cm, 최대 {max(errs) * 100:.1f}cm')
        lats = [abs(r[5]) for r in self.rows if not math.isnan(r[5])]
        if lats:
            out.append(f'옆 오차(차선에서 벗어남) 평균 {sum(lats) / len(lats) * 100:.1f}cm, 최대 {max(lats) * 100:.1f}cm')
        faces = [r[3] for r in self.rows if '>' in r[0] and not math.isnan(r[3])]
        if faces:
            out.append(f'테이블 보기 {len(faces)}번, 방향 오차 평균 {sum(faces) / len(faces):.1f}°, 최대 {max(faces):.1f}°')
        out.append(f'총 {time.monotonic() - self.t0:.0f}초')
        if self.fails:
            out.append(f'실패 {len(self.fails)}번: ' + ' / '.join(self.fails))
        out.append('결과: ' + (f'실패 ({self.failed})' if self.failed else
                               ('끝까지 감 (실패한 다리는 건너뜀)' if self.fails else '끝까지 완료')))
        if self.log_base:
            out.append(f'로그: {self.log_base}.log / _trace.csv / _result.csv')
        print('\n'.join(out))
        if self.log_f:
            self.log_f.write('\n'.join(out) + '\n')
            self.log_f.close()
        if self.trace_f:
            self.trace_f.close()
        if self.log_base:
            with open(self.log_base + '_result.csv', 'w', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                w.writerow(['node', 'sec', 'err_m', 'yaw_err_deg', 'along_m', 'lat_m'])
                for r in self.rows:
                    w.writerow([r[0], f'{r[1]:.2f}'] + ['' if math.isnan(v) else f'{v:.4f}' for v in r[2:]])


def build_legs(g, route, step, start=None, face_at=None):
    """-> (legs, faces, yaw_end, spin_after)"""
    legs, cur = [], [route[0]]
    for a_, b_ in zip(route, route[1:]):
        p = g.path(a_, b_)
        if p is None:
            sys.exit(f'{a_} -> {b_} 길이 없음 (후진 구간 · 일방통행 확인)')
        cur += p[1:]
        if step > 0:
            legs.append(cur)
            cur = [b_]
    if step <= 0:
        legs.append(cur)

    faces = {}
    if face_at:   # 테이블 앞 노드에서 다리를 끊고, 그 자리에서 테이블 모서리마다 한 다리씩 (제자리 회전)
        split = []
        for leg in legs:
            cur = [leg[0]]
            for nid in leg[1:]:
                cur.append(nid)
                if nid in face_at:
                    for k, (t, (fx, fy)) in enumerate(face_at[nid]):
                        n = g.nodes[nid]
                        faces[len(split)] = (t, math.atan2(fy - n['y'], fx - n['x']))
                        split.append(cur if k == 0 else [nid])
                    cur = [nid]
            if len(cur) > 1:
                split.append(cur)
        legs = split

    yaw_end, spin_after = {}, None
    if start:     # 도크에서: 첫 다리 = [첫 노드] (도크는 안 넣음 -> fleet_agent가 leave 먼저), 끝 yaw = 다음 노드 쪽
        if start not in g.nodes:
            sys.exit(f'node_graph에 없는 시작 노드: {start}')
        if route[0] not in g.nodes[start].get('next', []):
            sys.exit(f'{start} -> {route[0]} 길이 없음 (시작 도크의 next 확인)')
        legs.insert(0, [route[0]])
        faces = {k + 1: v for k, v in faces.items()}
        if len(route) > 1:
            nxt = g.path(route[0], route[1])
            yaw_end[0] = heading(g.nodes[route[0]], g.nodes[nxt[1]])
        spin_after = 0
    return legs, faces, yaw_end, spin_after


def main():
    argv = remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser(description='서버 없이 node_graph 노드를 Nav2로 순회')
    ap.add_argument('route', nargs='*', help='지나갈 노드 (기본: 홀 N1~N13 지그재그)')
    ap.add_argument('--preset', choices=sorted(PRESETS), help='hall = D1 빠지기(교점 회전) -> 홀 N1~N13 + 테이블 보기')
    ap.add_argument('--step', type=float, default=0.0, help='목표 노드마다 멈춰 쉴 초 (0 = 안 멈추고 한 번에)')
    ap.add_argument('--loops', type=int, default=1)
    ap.add_argument('--graph', default='', help='node_graph.yaml (기본: fleet_msgs share/docs)')
    ap.add_argument('--tables', nargs='?', const='near', choices=['near', 'all'], default=None,
                    help='테이블 앞 노드마다 서서 테이블 모서리(precision_stops face) 보기: near(가장 가까운) / all')
    ap.add_argument('--stops', default='', help='precision_stops.yaml (기본: fleet_msgs share/docs)')
    ap.add_argument('--start', default='', help='시작 도크 (예: D1). 첫 Move에 안 넣음 -> fleet_agent가 빠지기 먼저')
    ap.add_argument('--spin', type=float, default=0.0, help='--start 뒤 첫 노드에서 제자리 회전 각도 (AMCL 맞춤)')
    ap.add_argument('--log-dir', default='~/tour_logs', help='로그 폴더 ("" = 안 남김)')
    ap.add_argument('--keep-going', action='store_true',
                    help='다리가 실패(CMD_FAILED · BLOCKED)해도 멈추지 말고 다음 다리로 (연속 3번이면 멈춤). hall은 기본으로 켬')
    a = ap.parse_args(argv)
    if a.preset:
        pr = PRESETS[a.preset]
        a.route = a.route or pr['route']
        a.start = a.start or pr['start']
        a.spin = a.spin or pr['spin']
        a.tables = a.tables or pr['tables']
        a.keep_going = True

    docs = os.path.join(get_package_share_directory('fleet_msgs'), 'docs')
    path = a.graph or os.path.join(docs, 'node_graph.yaml')
    g = Graph(path)
    route = (a.route or DEFAULT_ROUTE) * max(1, a.loops)
    bad = [n for n in route if n not in g.nodes]
    if bad:
        sys.exit(f'node_graph에 없는 노드: {bad}')
    face_at = {}    # {N: [(테이블, (x, y)), ...] 가까운 순}
    if a.tables:
        with open(a.stops or os.path.join(docs, 'precision_stops.yaml'), encoding='utf-8') as f:
            stops = yaml.safe_load(f) or {}
        for t, cfg in stops.items():
            if isinstance(cfg, dict) and cfg.get('method') == 'table' and str(t) in g.nodes:
                pt = cfg.get('face') or (g.nodes[str(t)]['x'], g.nodes[str(t)]['y'])   # T 좌표 = 테이블 모서리
                for n in (cfg.get('from') or {}):
                    face_at.setdefault(str(n), []).append((str(t), tuple(map(float, pt))))
        if not face_at:
            sys.exit('precision_stops.yaml에 테이블(method table)이 없음')
        for n, lst in face_at.items():
            lst.sort(key=lambda tf: math.hypot(tf[1][0] - g.nodes[n]['x'], tf[1][1] - g.nodes[n]['y']))
            if a.tables == 'near':
                del lst[1:]

    legs, faces, yaw_end, spin_after = build_legs(g, route, a.step, a.start or None, face_at)
    log_base = ''
    if a.log_dir:
        log_base = os.path.join(os.path.expanduser(a.log_dir), time.strftime('tour_%Y%m%d_%H%M%S'))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C 때 멈춤 명령을 보내고 끝내려고
    node = Tour(g, legs, a.step, faces, spin_after if a.spin > 0 else None, a.spin, log_base)
    node.yaw_end = yaw_end
    node.keep_going = a.keep_going
    node.log('코스: ' + (f'{a.start} (빠지기) -> ' if a.start else '') + ' -> '.join(route)
             + (f'  | {route[0]}에서 {a.spin:.0f}° 회전' if a.start and a.spin > 0 else ''))
    if faces:
        node.log('테이블 보기: ' + ', '.join(f'{legs[i][-1]}->{t}' for i, (t, _) in sorted(faces.items())))
    node.log(f'다리 {len(legs)}개: ' + ' | '.join(' '.join(lg) for lg in legs))
    node.log(f'node_graph {path}')
    if a.start:
        node.log(f'로봇이 {a.start} 안에 있어야 한다 (robot_system start_node:={a.start} use_docking:=true precision_stops:=...). '
                 'Ctrl+C = 멈추고 결과 출력')
    else:
        node.log(f'로봇이 시작 노드 {route[0]} 위에 있어야 한다 (멀면 처음 노드로 먼저 간다). Ctrl+C = 멈추고 결과 출력')
    time.sleep(1.0)               # fleet_agent 구독 연결 기다림
    t_wait = time.monotonic() + 3.0
    while time.monotonic() < t_wait and node.pose() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.log(f'시작 위치 {node.ptext(node.pose())}  {node.amcl_text()}')
    node.send_leg()
    timer = node.create_timer(0.2, node.tick)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        stop = Control()
        stop.command = Control.CANCEL   # 지금 경로 버리고 멈춤
        node.ctl.publish(stop)
        node.cmd_pub.publish(Twist())
        time.sleep(0.3)
        node.failed = 'Ctrl+C로 중단'
    node.destroy_timer(timer)
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""노드 순회 시험: 서버 없이 로봇 혼자 node_graph의 노드들을 Nav2로 차례로 지나가 본다.

fleet_agent에 /fleet/move를 직접 보내고 /fleet/event를 받아서, 노드마다 걸린 시간과
도착했을 때 실제 위치(TF)와 노드 좌표의 차이를 찍는다. 서버(main_service)는 끄고 쓴다.

  ros2 run fleet_agent node_tour                    # 기본 코스: 홀 N1~N13 지그재그 후 N1로
  ros2 run fleet_agent node_tour N1 N4 N13 N11 N1   # 지나갈 노드를 직접 (사이 노드는 길 찾기로 채움)
  ros2 run fleet_agent node_tour --step 2 N1 N4 N1  # 목표 노드마다 멈춰 2초 쉬고 다음으로
  ros2 run fleet_agent node_tour --loops 3          # 같은 코스 3번
  ros2 run fleet_agent node_tour --tables           # 테이블 앞 노드(N2~N10)마다 서서 가장 가까운 테이블 모서리를 봄
  ros2 run fleet_agent node_tour --tables all       # 그 노드가 맡은 테이블 모두 (N3 = T1, T2 차례로)

  --tables: 테이블 모서리 = precision_stops.yaml T의 face (fleet_agent가 테이블 도착지일 때 직전 노드에서 보는 점).
            그 노드에서 다리를 끊고 마지막 yaw = 모서리 쪽으로 보내서, 도착 뒤 방향 오차를 잰다

  - 후진 구간(node_graph reverse)과 도킹 노드는 목표로 적지 않으면 지나가지 않는다 (전진 주행만 시험)
  - 지나가는 노드의 yaw = 다음 노드 쪽, 마지막 노드 = node_graph에 yaw가 있으면 그 값, 없으면 들어온 방향
"""
import argparse
import math
import os
import sys
import time
from collections import deque

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from fleet_msgs.msg import Control, Move, RobotEvent, Waypoint
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener

DEFAULT_ROUTE = ['N1', 'N2', 'N3', 'N4', 'N7', 'N6', 'N5', 'N8', 'N9', 'N10',
                 'N13', 'N12', 'N11', 'N8', 'N5', 'N2', 'N1']


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


class Tour(Node):
    def __init__(self, graph, legs, step_sec, faces=None):
        super().__init__('node_tour')
        self.g = graph
        self.legs = legs          # [[노드, ...], ...] 한 다리 = Move 하나
        self.faces = faces or {}  # {다리 index: (테이블, yaw)} 다리 끝에서 테이블 모서리를 봄
        self.step_sec = step_sec
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Move, '/fleet/move', rel)
        self.ctl = self.create_publisher(Control, '/fleet/control', rel)
        self.create_subscription(RobotEvent, '/fleet/event', self.on_event, rel)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.cmd_id = int(time.time()) % 1000000 * 100 + 50
        self.leg = -1
        self.t_leg = self.t_node = 0.0
        self.rows = []            # (노드, 걸린 초, 위치 오차 m, 방향 오차 deg)
        self.failed = None
        self.done = False
        self.wait_until = None

    def send_leg(self):
        self.leg += 1
        if self.leg >= len(self.legs):
            self.done = True
            return
        nodes = self.legs[self.leg]
        m = Move()
        self.cmd_id += 1
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
            elif 'yaw' in n:
                w.yaw = float(n['yaw'])
            elif i > 0:
                w.yaw = heading(self.g.nodes[nodes[i - 1]], n)
            m.waypoints.append(w)
        self.t_leg = self.t_node = time.monotonic()
        self.pub.publish(m)
        self.get_logger().info(f'[{self.leg + 1}/{len(self.legs)}] Move {m.cmd_id}: {" ".join(nodes)}')

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def on_event(self, e):
        if e.cmd_id != self.cmd_id:
            return
        now = time.monotonic()
        if e.type == RobotEvent.NODE_REACHED:
            n = self.g.nodes.get(e.node_id)
            p = self.pose()
            err = dyaw = float('nan')
            if n and p:
                err = math.hypot(p[0] - n['x'], p[1] - n['y'])
                w = next((w for w in self.last_waypoints() if w[0] == e.node_id), None)
                if w:
                    dyaw = math.degrees(abs(math.atan2(math.sin(p[2] - w[1]), math.cos(p[2] - w[1]))))
            self.rows.append((e.node_id, now - self.t_node, err, dyaw))
            self.get_logger().info(f'  도착 {e.node_id:4s} {now - self.t_node:5.1f}초  위치 오차 {err * 100:5.1f}cm')
            self.t_node = now
        elif e.type == RobotEvent.CMD_DONE:
            self.get_logger().info(f'  다리 끝 {e.node_id} ({now - self.t_leg:.1f}초)')
            if self.leg in self.faces:
                table, want = self.faces[self.leg]
                p = self.pose()
                dyaw = float('nan') if p is None else math.degrees(
                    abs(math.atan2(math.sin(p[2] - want), math.cos(p[2] - want))))
                self.rows.append((f'{e.node_id}>{table}', now - self.t_leg, float('nan'), dyaw))
                self.get_logger().info(f'  {e.node_id} -> {table} 모서리 보기: 방향 오차 {dyaw:.1f}°')
            if self.step_sec > 0 and self.leg + 1 < len(self.legs):
                self.wait_until = now + self.step_sec
            else:
                self.send_leg()
        elif e.type in (RobotEvent.CMD_FAILED, RobotEvent.BLOCKED):
            kind = 'CMD_FAILED' if e.type == RobotEvent.CMD_FAILED else 'BLOCKED'
            self.failed = f'{kind} {e.node_id} {e.reason}'
            self.get_logger().error(f'  {self.failed} -> 순회 멈춤')
            self.done = True

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
        print('\n==== 노드 순회 결과 ====')
        print(f'{"노드":6s}{"초":>7s}{"위치오차cm":>12s}{"방향오차deg":>13s}')
        for nid, sec, err, dyaw in self.rows:
            d = '' if math.isnan(dyaw) else f'{dyaw:.0f}'
            e = '-' if math.isnan(err) else f'{err * 100:.1f}'
            print(f'{nid:7s}{sec:7.1f}{e:>12s}{d:>13s}')
        errs = [r[2] for r in self.rows if not math.isnan(r[2])]
        if errs:
            print(f'도착 {len(errs)}개, 평균 오차 {sum(errs) / len(errs) * 100:.1f}cm, 최대 {max(errs) * 100:.1f}cm')
        faces = [r[3] for r in self.rows if '>' in r[0] and not math.isnan(r[3])]
        if faces:
            print(f'테이블 보기 {len(faces)}번, 방향 오차 평균 {sum(faces) / len(faces):.1f}°, 최대 {max(faces):.1f}°')
        print('결과: ' + (f'실패 ({self.failed})' if self.failed else '끝까지 완료'))


def main():
    argv = remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser(description='서버 없이 node_graph 노드를 Nav2로 순회')
    ap.add_argument('route', nargs='*', help='지나갈 노드 (기본: 홀 N1~N13 지그재그)')
    ap.add_argument('--step', type=float, default=0.0, help='목표 노드마다 멈춰 쉴 초 (0 = 안 멈추고 한 번에)')
    ap.add_argument('--loops', type=int, default=1)
    ap.add_argument('--graph', default='', help='node_graph.yaml (기본: fleet_msgs share/docs)')
    ap.add_argument('--tables', nargs='?', const='near', choices=['near', 'all'], default=None,
                    help='테이블 앞 노드마다 서서 테이블 모서리(precision_stops face) 보기: near(가장 가까운) / all')
    ap.add_argument('--stops', default='', help='precision_stops.yaml (기본: fleet_msgs share/docs)')
    a = ap.parse_args(argv)

    docs = os.path.join(get_package_share_directory('fleet_msgs'), 'docs')
    path = a.graph or os.path.join(docs, 'node_graph.yaml')
    g = Graph(path)
    face_at = {}    # {N: [(테이블, (x, y)), ...] 가까운 순}
    if a.tables:
        with open(a.stops or os.path.join(docs, 'precision_stops.yaml'), encoding='utf-8') as f:
            stops = yaml.safe_load(f) or {}
        for t, cfg in stops.items():
            if isinstance(cfg, dict) and cfg.get('method') == 'table' and cfg.get('face'):
                for n in (cfg.get('from') or {}):
                    face_at.setdefault(str(n), []).append((str(t), tuple(map(float, cfg['face']))))
        if not face_at:
            sys.exit('precision_stops.yaml에 face 있는 테이블이 없음')
        for n, lst in face_at.items():
            lst.sort(key=lambda tf: math.hypot(tf[1][0] - g.nodes[n]['x'], tf[1][1] - g.nodes[n]['y']))
            if a.tables == 'near':
                del lst[1:]
    route = (a.route or DEFAULT_ROUTE) * max(1, a.loops)
    bad = [n for n in route if n not in g.nodes]
    if bad:
        sys.exit(f'node_graph에 없는 노드: {bad}')

    # 목표 노드 사이를 길 찾기로 채운다. --step이면 목표마다 Move 하나, 아니면 전체를 Move 하나로
    legs, cur = [], [route[0]]
    for a_, b_ in zip(route, route[1:]):
        p = g.path(a_, b_)
        if p is None:
            sys.exit(f'{a_} -> {b_} 길이 없음 (후진 구간 · 일방통행 확인)')
        cur += p[1:]
        if a.step > 0:
            legs.append(cur)
            cur = [b_]
    if a.step <= 0:
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

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C 때 멈춤 명령을 보내고 끝내려고
    node = Tour(g, legs, a.step, faces)
    print('코스:', ' -> '.join(route))
    if faces:
        print('테이블 보기:', ', '.join(f'{legs[i][-1]}->{t}' for i, (t, _) in sorted(faces.items())))
    print('로봇이 시작 노드', route[0], '위에 있어야 한다 (멀면 처음 노드로 먼저 간다). Ctrl+C = 멈추고 결과 출력')
    time.sleep(1.0)               # fleet_agent 구독 연결 기다림
    node.send_leg()
    timer = node.create_timer(0.2, node.tick)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        stop = Control()
        stop.command = Control.CANCEL   # 지금 경로 버리고 멈춤
        node.ctl.publish(stop)
        time.sleep(0.3)
        node.failed = 'Ctrl+C로 중단'
    node.destroy_timer(timer)
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

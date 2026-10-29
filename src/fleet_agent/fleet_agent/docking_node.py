"""Docking 노드: fleet_agent가 맡긴 정밀 정차를 한다

fleet_agent <-> docking 인터페이스 (std_msgs/String, JSON)
  /docking/request  {"id", "node", "method": "dock" | "align" | "tape" | "dock_in" | "corridor", ...precision_stops.yaml 값}
                    fleet_agent가 붙이는 값: from_node, corridor는 nodes · then (아래)
  /docking/result   {"id", "node", "ok": bool, "detail": str, corridor면 "reached": 마지막 노드, "then_done": bool}
  /docking/cancel   {"id"}   -> 바로 멈추고 결과는 보내지 않는다
  /docking/progress {"id", "node"}            corridor: 가로선(또는 거리)으로 노드를 지날 때마다 (안 섬)
  /docking/extend   {"id", "nodes", "then"}   corridor: 달리는 중에 서버 APPEND로 길이 늘어남 -> 이어서 감

방법 -> 구현
  dock  : docking/marker_dock.py  Docker.dock(마커 marker의 목표 파일) -> back(m)만큼 후진   (D1)
  align : front/axis가 있으면 docking/corridor.py Steps.n14_align                             (N14: 마커 4 -> 우90 -> 5)
          없으면 예전 방식 (markers를 Docker.dock으로 차례로)
  tape  : docking/line_dock.py    LineDocker.run  E자 테이프로 바구니를 픽업대에              (P1)
          corridor=True면 먼저 corridor.follow_walls로 가로선 앞까지 (라이다 벽과 평행하게 후진)
  dock_in : corridor.Steps.dock_in  통로(벽 평행) -> N15 -> 우90 -> 마커로 후면 주차         (통로 쪽 도크)
  corridor: corridor.follow_bars    축 고정 후진을 벽 평행으로 nodes 끝까지. 가로선(N15 · N16)마다 progress
            then = tape(P1) -> LineDocker.run / dock_in(D2 · D3) -> 도크 줄 선에서 Steps.park_from_line
  (leave = 노드를 떠날 때, precision_stops의 <노드>.leave)
  undock    : docking/undock.py   마커로 나올 자리까지 + 회전 (+ align 있으면 방위각 정렬)    (D2/D3 -> N16/N15)
  table     : docking/table_dock.py TableDocker.run   다리 마커로 차선 정렬 -> 바닥 정지선            (T1~T6)
  table_exit: docking/table_dock.py TableDocker.leave 서빙 뒤 dist만큼 전진 (그다음 Nav2)
  tape_exit : docking/line_dock.py LineDocker.leave  앞으로 빼서 새 선에서 정지              (P1 -> 그다음 Nav2)
  마커 목표 파일은 혼자 실행으로 미리 저장: ros2 run fleet_agent marker_dock --record --id <마커>
  통로 · 도크 기록은 ros2 run fleet_agent dock_record all (docking/dock_record.py)

- 정밀 정차는 시간이 걸리고 멈춰서 재는 동작이라 따로 스레드에서 돈다 (TF는 노드 executor가 받음)
- timeout_sec (요청에 timeout이 있으면 그 값) 안에 못 끝내면 멈추고 ok=True, detail="timeout"으로 끝낸다 (Nav2 위치 인정)
- 일하는 동안만 /cmd_vel을 쓴다 (fleet_agent는 그동안 멈춰 있음)
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt16MultiArray
from tf2_ros import Buffer, TransformListener

from .docking import marker_dock as md
from .docking.corridor import BAR_BLACK, FOLLOW_STOP_EARLY, Lidar, Steps, _load, bar_update
from .docking.line_dock import DEFAULTS, LineDocker, Motion
from .docking.undock import undock
from .docking.table_dock import TableDocker


class DockingNode(Node):
    def __init__(self):
        super().__init__('docking')
        self.timeout = self.declare_parameter('timeout_sec', 40.0).value
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.result_pub = self.create_publisher(String, '/docking/result', reliable)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/docking/request', self.on_request, reliable)
        self.create_subscription(String, '/docking/cancel', self.on_cancel, reliable)
        self.progress_pub = self.create_publisher(String, '/docking/progress', reliable)
        self.create_subscription(String, '/docking/extend', self.on_extend, reliable)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.ir = {'v': None, 'n': 0}      # 바닥 센서 (tape · tape_exit)
        self.create_subscription(UInt16MultiArray, self.declare_parameter('ir_topic', '/ir_sensor/range').value,
                                 self.on_ir, 10)
        self.lidar = Lidar(self, self.tf_buf)      # 통로 벽 (corridor · tape corridor · dock_in)

        self.cam = None            # 처음 마커 도킹 때 열고 계속 씀
        self.job = None            # 지금 요청
        self.job_start = 0.0
        self.cancel_flag = False
        self.timed_out = False
        self.plan = None           # corridor: {'nodes', 'then'} (extend로 바뀜)
        self.info = {}             # 결과에 붙일 값 (corridor reached · then_done)
        self.lock = threading.Lock()
        self.create_timer(0.5, self.watchdog)
        self.get_logger().info('docking 노드 시작')

    # ------------------------------------------------------------ 요청
    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            if self.job is not None:
                self.publish(req, False, f'다른 정밀 정차 중 ({self.job.get("node")})')
                return
            self.job = req
            self.info = {}
            self.plan = {'nodes': list(req.get('nodes') or []), 'then': req.get('then')}
            self.job_start = time.monotonic()
            self.cancel_flag = False
            self.timed_out = False
        self.get_logger().info(f'{req.get("node")} {req.get("method")} 시작 {req}')
        threading.Thread(target=self.run, args=(req,), daemon=True).start()

    def on_ir(self, msg):
        v = list(msg.data[:3])
        self.ir['v'] = v
        self.ir['n'] += 1
        bar_update(self.ir, v)     # 통로 가로선 세기 (corridor)

    def on_extend(self, msg):
        try:
            r = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            if self.job is None or r.get('id') != self.job.get('id') or self.job.get('method') != 'corridor':
                return
            self.plan = {'nodes': list(r.get('nodes') or []), 'then': r.get('then')}
            self.job_start = time.monotonic()        # 길이 늘었으니 시간도 새로
        self.get_logger().info(f'통로 이어 감: {self.plan["nodes"]} then {(r.get("then") or {}).get("method")}')

    def progress(self, node):
        with self.lock:
            jid = self.job.get('id') if self.job else None
        self.progress_pub.publish(String(data=json.dumps({'id': jid, 'node': node})))
        self.get_logger().info(f'통로 {node} 지남')

    def on_cancel(self, msg):
        with self.lock:
            if self.job is not None:
                self.cancel_flag = True
                self.get_logger().info(f'{self.job.get("node")} 취소')

    def watchdog(self):
        with self.lock:
            limit = float(self.job.get('timeout', self.timeout)) if self.job is not None else self.timeout
            if self.job is not None and not self.cancel_flag and time.monotonic() - self.job_start > limit:
                self.timed_out = True
                self.cancel_flag = True
                self.get_logger().warn(f'{self.job.get("node")} {limit:.0f}초 넘김 -> 멈추고 Nav2 위치 인정')

    def cancelled(self):
        return self.cancel_flag

    # ------------------------------------------------------------ 실행 (스레드)
    def run(self, req):
        method = req.get('method')
        try:
            if method == 'dock':
                result = self.run_dock(req)
            elif method == 'align':
                result = self.run_align(req)
            elif method == 'tape':
                result = self.run_tape(req)
            elif method == 'dock_in':
                result = self.run_dock_in(req)
            elif method == 'corridor':
                result = self.run_corridor(req)
            elif method == 'tape_exit':
                result = LineDocker(self, self.cmd_pub, self.cancelled, self.tf_buf, self.ir).leave(req)
            elif method == 'undock':
                result = self.run_undock(req)
            elif method == 'table':
                result = self.run_table(req)
            elif method == 'table_exit':
                result = TableDocker(self, self.cmd_pub, self.cancelled, self.tf_buf, None, self.ir).leave(req)
            else:
                result = f'실패: 모르는 method {method}'
        except md.Cancelled:
            result = None
        except Exception as e:  # 카메라 · TF 문제 등
            result = f'실패: {e}'
        finally:
            self.cmd_pub.publish(Twist())
            md.close_log()

        with self.lock:
            timed_out = self.timed_out
            info = dict(self.info)
            self.job = None
        if result is None and not timed_out:
            return  # 취소: 결과 안 보냄
        if timed_out:
            self.publish(req, True, 'timeout: Nav2 위치 인정', info)
        else:
            self.publish(req, result == '성공', result, info)

    def docker(self, marker_id, tag):
        md.load_cam_file()
        md.open_log(tag)
        if self.cam is None:
            self.cam = md.Camera(marker_id)  # picamera2 (처음 1초 걸림)
        self.cam.id = marker_id
        return md.Docker(self, self.cmd_pub, self.tf_buf, self.cam, moving=True, spin=False,
                         cancelled=self.cancelled)

    def run_dock(self, req):
        marker = int(req['marker'])
        dk = self.docker(marker, f'{req.get("node")}_dock')
        dk.wait_pose()
        result = dk.dock(md.target_file_of(marker))
        back = float(req.get('back', 0.0))
        if result == '성공' and back > 0.0:
            left = dk.back_off(back)
            md.log(f'[후진] {back * 100:.1f} cm (남은 {left * 1000:+.1f} mm)')
        return result

    def run_table(self, req):
        """테이블: 들어온 노드(from_node)의 마커로 차선 정렬 -> 바닥 정지선 (docking/table_dock.py)"""
        f = (req.get('from') or {}).get(req.get('from_node'))
        if f is None:
            return f'실패: {req.get("node")} from에 {req.get("from_node")} 없음'
        cam = self.docker(int(f['marker']), f'{req.get("node")}_table').cam
        return TableDocker(self, self.cmd_pub, self.cancelled, self.tf_buf, cam, self.ir).run(req)

    def run_undock(self, req):
        dk = self.docker(int(req['marker']), f'{req.get("node")}_undock')
        return undock(self, self.cmd_pub, self.cancelled, self.tf_buf, dk.cam, req)

    def steps(self, marker, tag):
        """corridor.Steps (마커 + 라이다 + 바닥 센서). marker가 None이면 카메라 없이 (벽 따라가기만)"""
        if marker is None:
            md.open_log(tag)
            cam = None
        else:
            cam = self.docker(int(marker), tag).cam
        mv = Motion(self, self.cmd_pub, self.cancelled, self.tf_buf, self.ir, dict(DEFAULTS, ir_black=BAR_BLACK))
        mv.dk.wait_pose()
        return Steps(mv, cam, md.log, self.lidar)

    def corridor_names(self):
        """지금 계획에서 벽 따라 지나갈 통로 노드 (dock_in이면 도크 바로 앞 줄 노드까지)"""
        with self.lock:
            plan = dict(self.plan or {})
        ns = list(plan.get('nodes') or [])
        if (plan.get('then') or {}).get('method') == 'dock_in':
            ns = ns[:-1]
        return ns

    def run_corridor(self, req):
        """통로: 가로선 따라 후진 (progress) -> [then: tape = P1 삽입 / dock_in = 우90 -> 후면 주차]"""
        st = self.steps(None, f'{req.get("node")}_corridor')
        start = req.get('from_node', '')
        names = self.corridor_names()
        reached, why = st.follow_bars(self.corridor_names, start, on_pass=self.progress, resume=bool(req.get('resume')))
        with self.lock:
            plan = dict(self.plan or {})
            self.info = {'reached': reached, 'then_done': False}
        then = plan.get('then')
        names = self.corridor_names()
        if not then or (names and reached != names[-1]):
            return '성공'
        node = (plan.get('nodes') or [reached])[-1]
        if then.get('method') == 'tape':
            md.log(f'[{node}] tape: 가로선 -> X -> 시계 90 -> 동 선까지 삽입')
            result = LineDocker(self, self.cmd_pub, self.cancelled, self.tf_buf, self.ir).run(
                dict(then, approach=True, approach_max=float(then.get('approach_max', 0.6))))
        elif then.get('method') == 'dock_in':
            st = self.steps(int(then['marker']), f'{node}_dock_in')
            st.park_from_line(then)
            result = '성공'
        else:
            return f'실패: 모르는 then {then.get("method")}'
        with self.lock:
            self.info = {'reached': node, 'then_done': True}
        return result

    def run_tape(self, req):
        if req.get('corridor') and float(req.get('travel', 0.0)) > FOLLOW_STOP_EARLY:
            st = self.steps(None, f'{req.get("node")}_corridor')
            st.follow_walls(float(req['travel']), f'통로 -> {req.get("node")}')   # 가로선 앞 (또는 가로선)까지
            req = dict(req, approach=True, approach_max=float(req.get('approach_max', 0.8)))
        return LineDocker(self, self.cmd_pub, self.cancelled, self.tf_buf, self.ir).run(req)

    def run_dock_in(self, req):
        st = self.steps(int(req['marker']), f'{req.get("node")}_dock_in')
        st.dock_in(req, travel=float(req.get('travel', 0.0)) if req.get('corridor') else 0.0)
        return '성공'

    def run_align(self, req):
        if 'front' in req:                 # 새 N14 정렬: 마커 4 -> 우90 -> 마커 5 (corridor.Steps.n14_align)
            st = self.steps(_load(f'~/align_{req["front"]}.json')['id'], f'{req.get("node")}_align')
            st.n14_align(_load(f'~/align_{req["front"]}.json'), _load(f'~/align_{req["axis"]}.json'),
                         float(req.get('turn_deg', -90.0)))
            return '성공'
        result = '실패: markers 없음'
        for marker in req.get('markers', []):
            dk = self.docker(int(marker), f'{req.get("node")}_align{marker}')
            dk.wait_pose()
            if not dk.find_marker():
                return f'실패: 마커 {marker}가 안 보임'
            result = dk.dock(md.target_file_of(int(marker)))
            if result != '성공':
                return f'마커 {marker}: {result}'
        return result

    # ------------------------------------------------------------
    def publish(self, req, ok, detail, info=None):
        out = {'id': req.get('id'), 'node': req.get('node'), 'ok': ok, 'detail': detail}
        out.update(info or {})
        self.result_pub.publish(String(data=json.dumps(out, ensure_ascii=False)))
        self.get_logger().info(f'{req.get("node")} 끝: {detail}')

    def close(self):
        self.cmd_pub.publish(Twist())
        if self.cam is not None:
            self.cam.close()


def main():
    rclpy.init()
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

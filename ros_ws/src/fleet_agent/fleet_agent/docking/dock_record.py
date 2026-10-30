#!/usr/bin/env python3
"""주방 통로 · 도크 기록 도구 (혼자 실행. fleet_agent는 끄고 bringup + 바닥 센서 + camera_node만 켜고)
  카메라는 camera_node 영상을 구독 (ros2 run fleet_agent camera_node). camera_node 없이 직접 열려면 --direct

  ros2 run fleet_agent dock_record all            # D3에 손으로 반듯하게 넣고: 로봇이 직접 d3 · n15 · n15s · n14s · n14e
  ros2 run fleet_agent dock_record dock --dock d2 # 도크 쪽만 (d2 · n16 · n16s)
  ros2 run fleet_agent dock_record align <이름> --id <마커> [--size 0.05]   # 그 자리에서 정렬 기록 하나
  ros2 run fleet_agent dock_record check [--dock d3]   # 기록 확인 (cm · °)
  ros2 run fleet_agent dock_record scan                # 지금 자리에서 라이다로 양옆 벽이 보이나
  ros2 run fleet_agent dock_record d1 [--line-tz 0.21] # 입구 D1 칸에 손으로 반듯하게 (앞 = 마커 1, 바구니 = 칸막이):
                                                       #   D1 · 교점(마커 1 · 6) · N1 (docking/entrance.py). 끝: N1에서 남쪽
  ros2 run fleet_agent dock_record check --dock d1     # D1 기록 확인

all 순서 (손으로 놓는 건 D3 하나. 나머지는 로봇이 벽과 수직으로 맞추고 찍음)
  D3(손) -> 앞으로 곧게 나와 마커 3이 --n15-tz(기본 15 cm)인 곳 = N15: n15 저장
         -> d3 = n15에서 나온 거리만큼 곧게 뒤 (도크 안 34 cm에서는 마커 기울기가 몇 °씩 틀려서 안 씀)
         -> 좌90(남): n15s (마커 5) -> 축 그대로 전진해 마커 5가 --n14-tz(기본 13 cm)인 곳 = N14:
            남쪽 벽 수직 정렬 후 n14s -> 좌90(동): 동쪽 벽 수직 정렬 후 n14e.  끝나면 N14에서 동쪽을 봄
파일 형식: fleet_msgs/docs/record_json_format.md
"""
import argparse
import math
import os

from . import marker_dock as md
from . import entrance, records
from .corridor import Lidar, Steps, _save, align_record, bearing, check, side_lines
from .line_dock import DEFAULTS, Motion
from .undock import _set_size

# 도크별 기록 이름 (precision_stops.yaml의 dock_in 설정과 같아야 함). 도크 이름은 최종 맵 기준
RECORDS = {
    'd3': {'front': 'n14e', 'axis': 'n14s', 'via': 'n15s', 'stop': 'n15', 'marker': 3, 'target': 'd3'},
    'd2': {'front': 'n14e', 'axis': 'n14s', 'via': 'n16s', 'stop': 'n16', 'marker': 2, 'target': 'd2'},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['all', 'dock', 'align', 'check', 'scan', 'd1'])
    ap.add_argument('name', nargs='?', help='align: 기록 이름 (~/align_<이름>.json)')
    ap.add_argument('--dock', default='d3', choices=sorted(RECORDS) + ['d1'])
    ap.add_argument('--line-tz', type=float, default=entrance.LINE_TZ, help='d1: D1에서 곧게 앞으로 교점까지 대략 (마커 1 거리 m)')
    ap.add_argument('--id', type=int, help='align: 마커 ID')
    ap.add_argument('--size', type=float, default=0.05, help='align: 마커 한 변 m (기본 5 cm)')
    ap.add_argument('--n15-tz', type=float, default=0.15, help='all/dock: N15 = 마커 3까지 카메라 거리 m')
    ap.add_argument('--n14-tz', type=float, default=0.13, help='all: N14 = 마커 5(남쪽 벽)까지 카메라 거리 m')
    ap.add_argument('--direct', action='store_true',
                    help='camera_node 없이 카메라를 직접 엶 (camera_node · docking 노드가 꺼져 있어야 함)')
    args, _ = ap.parse_known_args()
    if args.cmd == 'check' and args.dock == 'd1':
        print(entrance.check())
        return
    cfg = RECORDS.get(args.dock, RECORDS['d3'])
    if args.cmd == 'check':
        print(check((cfg['front'], cfg['axis'], cfg['via']), (cfg['stop'], cfg['target'])))
        return
    if args.cmd == 'align' and (not args.name or args.id is None):
        raise SystemExit('align 에는 이름과 --id 가 필요합니다: dock_record align n14e --id 4')

    import threading
    import time

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import UInt16MultiArray
    from tf2_ros import Buffer, TransformListener

    md.load_cam_file()
    md.open_log(f'record_{args.cmd}')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('dock_record')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    tf_buf = Buffer()
    tf_listener = TransformListener(tf_buf, node)  # noqa: F841
    ir = {'v': None, 'n': 0}

    def on_ir(msg):
        ir['v'] = list(msg.data[:3])
        ir['n'] += 1
    node.create_subscription(UInt16MultiArray, '/ir_sensor/range', on_ir, 10)
    lidar = Lidar(node, tf_buf)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def open_cam(marker):
        # 기본: camera_node의 /camera/image_raw 구독 (executor 스레드가 영상을 받음). --direct = 예전처럼 직접 엶
        return md.Camera(marker) if args.direct else md.Camera(marker, node, spin=False)
    cam = None
    mv = Motion(node, pub, None, tf_buf, ir, dict(DEFAULTS))
    result = None
    try:
        mv.dk.wait_pose()
        if args.cmd == 'scan':
            got, t_end = None, time.time() + 3.0
            while time.time() < t_end:
                q = lidar.take()
                if q is not None:
                    got = q
                time.sleep(0.05)
            if got is None:
                result = '실패: /scan 이 안 들어옴 (bringup 켰나요?)'
            else:
                fl = side_lines(got)
                for side in ('왼쪽', '오른쪽'):
                    if side in fl:
                        d_, a_, n_, e_ = fl[side]
                        md.log(f'[라이다] {side} 벽: {d_ * 100:.1f} cm, 각 {math.degrees(a_):+.1f}°, 점 {n_}개, '
                               f'흩어짐 {e_ * 1000:.1f} mm')
                    else:
                        md.log(f'[라이다] {side} 벽: 안 보임')
                result = '확인 끝 (로봇이 남쪽을 보면 왼쪽 = 동쪽 주방, 오른쪽 = 서쪽 칸막이)'
        elif args.cmd == 'align':
            cam = open_cam(args.id)
            _set_size(args.size)
            m = cam.measure_stable(frames=30)
            if m is None:
                result = f'실패: 마커 {args.id}가 안 보임'
            else:
                path = records.save_path(f'align_{args.name}.json')
                _save(path, align_record(args.id, args.size, m))
                result = (f'저장: 거리 {m["tz"] * 100:.1f} cm, 방위각 {math.degrees(bearing(m)):+.2f}°, '
                          f'기울기 {math.degrees(m["yaw"]):+.1f}° -> {path}')
        elif args.cmd == 'd1':
            cam = open_cam(1)
            result = entrance.Entrance(mv, cam, md.log, lidar, line_tz=args.line_tz).record()
        else:
            cam = open_cam(cfg['marker'])
            st = Steps(mv, cam, md.log, lidar)
            result = st.record_all(cfg, args.n15_tz, args.n14_tz, dock_only=(args.cmd == 'dock'))
    except KeyboardInterrupt:
        result = '중단: Ctrl+C'
    except Exception as e:
        result = f'실패: {e}'
    finally:
        try:
            mv.stop(0.1)
        except Exception:
            pass
        _set_size(0.025)
        ex.shutdown()
        th.join(timeout=1.0)
        if cam is not None:
            cam.close()
        md.log(f'결과: {result}')
        md.close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

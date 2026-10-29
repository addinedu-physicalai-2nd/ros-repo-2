#!/usr/bin/env python3
"""주방 통로 · 도크 기록 도구 (혼자 실행. fleet_agent는 끄고 bringup + 바닥 센서만 켜고)

  ros2 run fleet_agent dock_record all            # D3에 손으로 반듯하게 넣고: 로봇이 직접 d3 · n15 · n15s · n14s · n14e
  ros2 run fleet_agent dock_record dock --dock d2 # 도크 쪽만 (d2 · n16 · n16s)
  ros2 run fleet_agent dock_record align <이름> --id <마커> [--size 0.05]   # 그 자리에서 정렬 기록 하나
  ros2 run fleet_agent dock_record check [--dock d3]   # 기록 확인 (cm · °)
  ros2 run fleet_agent dock_record scan                # 지금 자리에서 라이다로 양옆 벽이 보이나

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
    ap.add_argument('cmd', choices=['all', 'dock', 'align', 'check', 'scan'])
    ap.add_argument('name', nargs='?', help='align: 기록 이름 (~/align_<이름>.json)')
    ap.add_argument('--dock', default='d3', choices=sorted(RECORDS))
    ap.add_argument('--id', type=int, help='align: 마커 ID')
    ap.add_argument('--size', type=float, default=0.05, help='align: 마커 한 변 m (기본 5 cm)')
    ap.add_argument('--n15-tz', type=float, default=0.15, help='all/dock: N15 = 마커 3까지 카메라 거리 m')
    ap.add_argument('--n14-tz', type=float, default=0.13, help='all: N14 = 마커 5(남쪽 벽)까지 카메라 거리 m')
    args, _ = ap.parse_known_args()
    cfg = RECORDS[args.dock]
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
            cam = md.Camera(args.id)
            _set_size(args.size)
            m = cam.measure_stable(frames=30)
            if m is None:
                result = f'실패: 마커 {args.id}가 안 보임'
            else:
                path = os.path.expanduser(f'~/align_{args.name}.json')
                _save(path, align_record(args.id, args.size, m))
                result = (f'저장: 거리 {m["tz"] * 100:.1f} cm, 방위각 {math.degrees(bearing(m)):+.2f}°, '
                          f'기울기 {math.degrees(m["yaw"]):+.1f}° -> {path}')
        else:
            cam = md.Camera(cfg['marker'])
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

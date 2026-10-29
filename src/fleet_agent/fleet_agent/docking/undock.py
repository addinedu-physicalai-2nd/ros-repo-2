"""마커 언도킹 + 회전 + 방향 정렬 (D3/D2 -> N15/N16, 주방 통로 후진 준비)

docking 노드가 부른다:  undock(node, pub, cancelled, tf_buf, cam, req) -> '성공' / '실패: ...'
  req (precision_stops.yaml의 D3.leave 등):
    marker   : 도크 마커 ID (로봇 앞 벽)
    target   : 나올 자리 이름 -> ~/dock_target_<target>.json   (marker_dock --id <marker> --name <target> --record)
    turn_deg : 나온 뒤 제자리 회전 (+ 왼쪽). 기본 90 (남쪽 보기 = 후진 방향이 북쪽)
    align    : 방향 정렬 기준 이름 -> ~/align_<align>.json     (marker_align --id <ID> --size 0.05 --name <align> --record)
               없으면 정렬 생략

순서 (멈춰 재고 odom으로 움직임, 카메라는 docking 노드가 열어 둔 것을 씀)
  1. 도크에서 마커를 한 번 잼. 멀면(22 cm+) 각도는 안 믿고 로봇이 벽을 정면으로 본다고 봄 (도킹 직후라)
  2. 나올 자리까지 한 번에 (가감속, 곡선) -> 마커를 정면으로 보고 다시 잼 -> 앞뒤 1 cm 넘게 어긋나면 고침
  3. turn_deg 회전 (나올 자리 방향 맞추기와 합쳐 회전 한 번)
  4. 정렬 마커의 '방위각'(화면 어느 쪽에 보이나)만 보고 제자리 회전. 멀어도 0.1° 수준으로 안정적
"""
import json
import math
import os
import time

import numpy as np

from . import marker_dock as md
from .line_dock import DEFAULTS, Motion

V_UNDOCK = 0.10
FIX_M = 0.010
FAR_ANGLE_M = 0.22
FRAMES = 8
ALIGN_FRAMES = 6
ALIGN_TOL_DEG = 0.5
ALIGN_VERIFY_DEG = 2.0
ALIGN_MAX_DEG = 20.0


def _set_size(size):
    md.MARKER_SIZE_M = size
    h = size / 2
    md.OBJ = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def _dock_est(m, p):
    if m['tz'] < FAR_ANGLE_M:
        return md.dock_in_odom(m, p)
    M = np.array([md.CAM_X_M + m['tz'], -m['tx']])
    return np.array(p[:2]) + md.rot(p[2]) @ M, md.wrap(p[2] + math.pi)


def _go_to(mv, x, y, vmax):
    """(x, y)까지 앞으로 한 번에 (가면서 방향을 곡선으로 맞추고 감속 도착)"""
    acc = DEFAULTS['accel']
    t0 = time.time()
    while time.time() - t0 < 20.0:
        p = mv.dk.pose()
        dx, dy = x - p[0], y - p[1]
        dist = math.hypot(dx, dy)
        if dist < max(0.003, abs(mv.last_v) * 0.06):
            break
        ang = md.wrap(math.atan2(dy, dx) - p[2])
        if dist < 0.02 and abs(ang) > math.radians(35):
            break                                   # 바로 옆으로 조금: 제자리에서 못 고침
        if abs(ang) > math.radians(35):
            mv.stop(0.0)
            mv.dk.rotate_to(math.atan2(dy, dx))
            continue
        v_t = min(vmax, math.sqrt(acc * dist) + 0.02) * max(0.0, math.cos(ang))
        v = md.clamp(min(v_t, mv.last_v + acc / 30.0), 0.0, vmax)
        mv.send(v, md.clamp(3.0 * ang, -0.8, 0.8) if dist > 0.01 else 0.0)
        time.sleep(1 / 30.0)
    mv.stop()


def undock(node, pub, cancelled, tf_buf, cam, req):
    log = node.get_logger().info
    mv = Motion(node, pub, cancelled, tf_buf, None, dict(DEFAULTS))
    mv.dk.wait_pose()
    marker = int(req['marker'])
    path = os.path.expanduser(f'~/dock_target_{req["target"]}.json')
    with open(path) as f:
        g = json.load(f)
    try:
        # ---- 1~2. 나올 자리까지
        _set_size(0.025)
        cam.id = marker
        mv.stop(0.15)
        m = cam.measure_stable(frames=FRAMES)
        if m is None:
            return f'실패: 도크 마커 {marker}가 안 보임'
        p = mv.dk.pose()
        D, phi = _dock_est(m, p)
        goal = md.dock_to_odom(D, phi, g['gx'], g['gy'], g['gth'])
        log(f'[undock] 마커 {m["tz"] * 100:.1f} cm -> {req["target"]}까지 '
            f'{math.hypot(goal[0] - p[0], goal[1] - p[1]) * 100:.1f} cm')
        _go_to(mv, goal[0], goal[1], V_UNDOCK)
        mv.dk.rotate_to(goal[2])
        mv.stop(0.12)
        h_end = goal[2]
        m2 = cam.measure_stable(frames=FRAMES, timeout=1.5)
        if m2 is not None:
            p2 = mv.dk.pose()
            D2, phi2 = _dock_est(m2, p2)
            goal2 = md.dock_to_odom(D2, phi2, g['gx'], g['gy'], g['gth'])
            fwd = (goal2[0] - p2[0]) * math.cos(p2[2]) + (goal2[1] - p2[1]) * math.sin(p2[2])
            log(f'[undock] 도착 확인: 앞뒤 {fwd * 1000:+.0f} mm')
            if abs(fwd) > FIX_M:
                mv.move_along(fwd, p2[2])
            h_end = goal2[2]

        # ---- 3. 회전
        turn = math.radians(float(req.get('turn_deg', 90.0)))
        if turn:
            e = mv.dk.rotate_to(md.wrap(h_end + turn))
            log(f'[undock] {math.degrees(turn):+.0f}° 회전 (남은 {e:+.1f}°)')

        # ---- 4. 방향 정렬 (방위각)
        name = req.get('align')
        if name:
            with open(os.path.expanduser(f'~/align_{name}.json')) as f:
                ref = json.load(f)
            _set_size(ref.get('size', 0.025))
            cam.id = ref['id']
            for k in range(1, 4):
                mv.stop(0.12)
                ma = cam.measure_stable(frames=ALIGN_FRAMES)
                if ma is None:
                    return f'실패: 정렬 마커 {ref["id"]}가 안 보임'
                err = math.atan2(-ma['tx'], md.CAM_X_M + ma['tz']) - ref['bearing']
                log(f'[undock] 정렬 {k}: 방향 오차 {math.degrees(err):+.2f}°')
                if abs(math.degrees(err)) > ALIGN_MAX_DEG:
                    return f'실패: 정렬 오차 {math.degrees(err):+.0f}° (기준 저장 자세 확인)'
                if abs(math.degrees(err)) < ALIGN_TOL_DEG:
                    break
                mv.dk.rotate_to(md.wrap(mv.dk.pose()[2] + err))
                if abs(math.degrees(err)) < ALIGN_VERIFY_DEG:
                    break
        return '성공'
    finally:
        _set_size(0.025)       # 도킹(dock)은 2.5 cm 마커 기준
        cam.id = marker

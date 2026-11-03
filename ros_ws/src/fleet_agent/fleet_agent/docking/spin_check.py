"""제자리 회전 시험: odom이 말하는 회전과 실제 회전(라이다 스캔 겹치기)을 비교 -> 바퀴 간격(wheel_separation) 보정 비율

  ros2 run fleet_agent dock_record spin [--turns 2] [--w 0.6]
  (fleet_agent 끄고 bringup만. 주변 0.8 m 안에 벽 · 칸막이가 보이는 자리, 사람은 비켜서)

원리: 처음 스캔 저장 -> 왼쪽으로 turns바퀴 (odom으로 잼) -> 멈춰서 스캔 -> 처음 스캔과 겹쳐 실제로 몇 ° 덜/더 돌았나
      -> 오른쪽으로도 똑같이 (양쪽 평균 = 바퀴 간격 비율, 양쪽 차이 = 바퀴 지름 차이 같은 한쪽 쏠림)
odom 회전각 = (오른 바퀴 - 왼 바퀴) / 바퀴 간격 이라서
  실제가 odom보다 덜 돌았으면 (odom이 부풀림) -> 바퀴 간격 설정이 실제보다 작음 -> 새 간격 = 지금 간격 x 비율
"""
import math
import time

import numpy as np

from . import scan_match as sm


def turn_by(mv, total, w, log):
    """odom으로 total rad 만큼 이어서 돎 (부호 = 방향). 반환 실제로 odom이 잰 회전 (rad, 이어 붙인 값)"""
    sgn = 1.0 if total > 0 else -1.0
    th_prev = mv.dk.pose()[2]
    acc = 0.0
    t_end = time.time() + abs(total) / max(w, 0.1) * 3 + 10
    while time.time() < t_end:
        th = mv.dk.pose()[2]
        acc += sm.wrap(th - th_prev)
        th_prev = th
        left = abs(total) - sgn * acc
        if left <= 0.002:
            break
        ww = max(0.15, min(w, 2.0 * left))
        mv.dk.send(0.0, sgn * ww)
        time.sleep(1 / 30.0)
    mv.stop(0.6)
    th = mv.dk.pose()[2]
    acc += sm.wrap(th - th_prev)
    return acc


def run(mv, lidar, log, turns=2, w=0.6, secs=1.5):
    out = {}
    for name, sgn in (('왼쪽(반시계)', 1.0), ('오른쪽(시계)', -1.0)):
        ref = sm.collect(lidar, secs, time.sleep)              # 방향마다 출발 스캔을 새로 (앞 방향 오차가 안 섞이게)
        if ref is None:
            raise RuntimeError('/scan 이 안 들어옴 (bringup 켰나요?)')
        ref = sm.prep(ref)
        odom = turn_by(mv, sgn * turns * 2 * math.pi, w, log)
        cur = sm.collect(lidar, secs, time.sleep)
        if cur is None:
            raise RuntimeError('회전 뒤 /scan 이 안 들어옴')
        # 로봇이 실제로 정답(처음) 자세에서 얼마나 틀어졌나 = 실제 회전 - (odom 회전을 360° 단위로 반올림한 것)
        resid_odom = sm.wrap(odom)                                  # odom 기준 처음 방향에서 남은 각 (거의 0)
        r = sm.icp_search(cur, ref, search_deg=60.0, step_deg=10.0, max_t=0.10)
        if not r.get('ok'):
            raise RuntimeError(f'{name}: 처음 스캔과 겹치기 실패 ({r.get("why")}) -> 벽이 보이는 자리에서 다시')
        real = odom - resid_odom + r['th']        # 실제 총 회전 = 정수 바퀴 + 실제로 남은 각
        ratio = real / odom
        out[name] = {'odom_deg': math.degrees(odom), 'real_deg': math.degrees(real), 'ratio': ratio,
                     'icp': r}
        log(f'[회전 시험] {name} {turns}바퀴: odom {math.degrees(odom):+.1f}°, 실제 {math.degrees(real):+.1f}° '
            f'(차이 {math.degrees(real - odom):+.1f}°, 실제/odom = {ratio:.4f}, 겹침 {r["inlier"] * 100:.0f}%, '
            f'자리 밀림 {math.hypot(r["x"], r["y"]) * 1000:.0f} mm)')
    rs = [v['ratio'] for v in out.values()]
    k = float(np.mean(rs))
    skew = math.degrees((out['왼쪽(반시계)']['real_deg'] - out['왼쪽(반시계)']['odom_deg'])
                        + (out['오른쪽(시계)']['real_deg'] - out['오른쪽(시계)']['odom_deg'])) / 2 / turns
    out['ratio'] = k
    out['per_turn_err_deg'] = (k - 1.0) * 360.0
    msg = (f'실제/odom = {k:.4f} (한 바퀴에 {(k - 1) * 360:+.1f}°, 90°에 {(k - 1) * 90:+.2f}°). '
           f'바퀴 간격 설정 x {1 / k:.4f} 로 바꾸기 권장 '
           + ('(차이 1% 안: 그대로 둬도 됨)' if abs(k - 1) < 0.01 else '')
           + f' | 왼/오 차이 {skew:+.2f}°/바퀴 (크면 바퀴 지름 차이)')
    out['msg'] = msg
    return out

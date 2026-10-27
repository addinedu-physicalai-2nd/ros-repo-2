#!/usr/bin/env python3
"""로봇 1대용 domain_bridge 설정 파일을 만든다.

사용:
  python3 make_bridge_config.py pinky1 1            # 서버 도메인 기본값 0
  python3 make_bridge_config.py pinky3 3 --server-domain 30

결과: 같은 폴더에 <robot_id>_bridge.yaml 생성
로봇 쪽 토픽:  /fleet/state, /fleet/event, /fleet/move, /fleet/control, /fleet/display  (namespace 없음)
서버 쪽 토픽:  /<robot_id>/fleet/...                                     (브리지가 이름 변환)
"""
import argparse
import os

TEMPLATE = """\
# 자동 생성: python3 make_bridge_config.py {rid} {rd} --server-domain {sd}
# 서버 도메인({sd}) <-> {rid} 도메인({rd}) 사이에서 fleet 토픽 5개만 전달한다.
# 로봇 쪽은 namespace 없이 /fleet/..., 서버 쪽은 /{rid}/fleet/... 로 이름을 바꿔 전달한다.
# Nav2, tf, 라이다 등 나머지 토픽은 {rid} 도메인 안에만 남는다.
#
# 실행: ros2 run domain_bridge domain_bridge {rid}_bridge.yaml
# (실행하는 터미널에서 fleet_msgs가 source 되어 있어야 한다)

name: {rid}_fleet_bridge
from_domain: {rd}     # 기본 방향: 로봇 -> 서버
to_domain: {sd}

topics:
  # ---- 로봇 -> 서버 ----
  /fleet/state:
    type: fleet_msgs/msg/RobotState
    remap: /{rid}/fleet/state
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1

  /fleet/event:
    type: fleet_msgs/msg/RobotEvent
    remap: /{rid}/fleet/event
    qos:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 10

  # ---- 서버 -> 로봇 (토픽별로 도메인 방향을 뒤집음) ----
  /{rid}/fleet/move:
    type: fleet_msgs/msg/Move
    from_domain: {sd}
    to_domain: {rd}
    remap: /fleet/move
    qos:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 10

  /{rid}/fleet/control:
    type: fleet_msgs/msg/Control
    from_domain: {sd}
    to_domain: {rd}
    remap: /fleet/control
    qos:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 10

  /{rid}/fleet/display:
    type: fleet_msgs/msg/Display
    from_domain: {sd}
    to_domain: {rd}
    remap: /fleet/display
    qos:
      reliability: reliable
      durability: transient_local   # 로봇(또는 브리지)이 늦게 켜져도 마지막 화면을 받음
      history: keep_last
      depth: 1
"""


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('robot_id', help='로봇 ID (예: pinky1). RobotState/RobotEvent의 robot_id와 같아야 함')
    p.add_argument('robot_domain', type=int, help='로봇의 ROS_DOMAIN_ID')
    p.add_argument('--server-domain', type=int, default=0, help='서버의 ROS_DOMAIN_ID (기본 0)')
    a = p.parse_args()
    if a.robot_domain == a.server_domain:
        p.error('로봇 도메인과 서버 도메인은 달라야 합니다')
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'{a.robot_id}_bridge.yaml')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(TEMPLATE.format(rid=a.robot_id, rd=a.robot_domain, sd=a.server_domain))
    print(f'생성: {out}')


if __name__ == '__main__':
    main()

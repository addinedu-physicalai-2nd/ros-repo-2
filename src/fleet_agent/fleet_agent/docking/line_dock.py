"""라인트레이싱 정밀 정차 (구조만, 추후 구현): P1 픽업대 · T1~T6 테이블

docking 노드가 부른다:  LineDocker(node, pub, cancelled).run(req) -> 결과 문구 ('성공'이면 성공)
  req = precision_stops.yaml 값 + node (예: {"node": "P1", "method": "tape"})
  - 일하는 동안만 /cmd_vel을 쓴다. 주기마다 cancelled()가 True면 바로 멈추고 Cancelled를 던진다
  - 바구니(뒤)를 픽업대 · 테이블 쪽으로 넣는 방향이라 후진으로 따라간다 (카메라는 앞을 봄)

TODO
  - 라인 센서 입력 (바닥 IR 센서 / 아래 보는 카메라 등, 정해지면 토픽 구독)
  - 라인 찾기 -> 따라가기 (좌우 오차 P제어) -> 끝 표시(가로 라인 · 거리)에서 멈춤
"""
from .marker_dock import Cancelled  # noqa: F401 (취소 예외를 같이 씀)


class LineDocker:
    def __init__(self, node, pub, cancelled):
        self.node = node
        self.pub = pub
        self.cancelled = cancelled

    def run(self, req):
        return '실패: tape(라인트레이싱) 미구현 - Nav2 위치 인정'

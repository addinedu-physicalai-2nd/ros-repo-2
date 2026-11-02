"""카메라로 바로 앞 손님 막기 (LiDAR에 안 잡히는 작은 레고 사람):  python3 -m pytest test/test_camera_block.py -v"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet_agent.core import CameraBlock, ObstacleClass, near_person  # noqa: E402


def test_near_person_big_and_center_only():
    assert near_person([(0.40, 0.40, 0.60, 0.90)])               # 크고 가운데 -> 막음
    assert not near_person([(0.45, 0.40, 0.55, 0.55)])           # 작음 (멀리) -> 안 막음
    assert not near_person([(0.85, 0.30, 0.99, 0.90)])           # 옆으로 비켜 섬 -> 안 막음
    assert near_person([(0.85, 0.3, 0.99, 0.9), (0.3, 0.2, 0.5, 0.8)])
    assert not near_person([])


def test_camera_block_hold_and_release():
    c = CameraBlock(hold=1.0)
    assert not c.active(0.0)
    c.update(0.0, True)
    assert c.active(0.5)
    assert not c.active(1.2), '결과가 끊기면 저절로 풀림'
    c.update(2.0, True)
    c.update(2.3, False)
    assert not c.active(2.3), 'false면 바로 풀림 (사람 치우면 곧장 출발)'


def test_camera_only_block_is_person():
    # LiDAR엔 안 잡혀도 (거리 모름 -> 멈춤 거리) 카메라로 막히면 PERSON
    cls = ObstacleClass()
    assert cls.update(0.0, 0.18, person=True) == 'PERSON'
    assert cls.update(0.3, 0.18) == 'PERSON'
    assert cls.update(0.4, None) == ''

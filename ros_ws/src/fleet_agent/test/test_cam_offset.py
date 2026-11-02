"""카메라 오프셋 보정 (marker_dock.cam_correct · load_cam_file · Camera.measure raw):  python3 -m pytest test/"""
import json
import math
import os
import socket
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
pytest.importorskip('cv2')

from fleet_agent.docking import marker_dock as md  # noqa: E402
from fleet_agent.docking import table_dock  # noqa: E402


def test_zero_offset_is_identity():
    m = {'tz': 0.3, 'tx': 0.01, 'yaw': 0.02}
    assert md.cam_correct(m, 0.0, 0.0) is m
    assert md.cam_correct(None, 0.01, 0.05) is None


def test_rotated_camera_maps_back_to_reference():
    # 기준 카메라 정면 0.33 m의 마커를, 반시계로 dyaw 틀어진 카메라가 보면 오른쪽으로 비껴 보임 -> 보정하면 정면
    z, dyaw, dy = 0.33, math.radians(4.578), -0.00404
    y2 = -z * math.sin(dyaw) - dy * math.cos(dyaw)
    z2 = z * math.cos(dyaw) - dy * math.sin(dyaw)
    seen = {'tz': z2, 'tx': -y2, 'yaw': -dyaw}
    out = md.cam_correct(seen, dy, dyaw)
    assert abs(out['tz'] - z) < 1e-9 and abs(out['tx']) < 1e-9 and abs(out['yaw']) < 1e-9
    # 보정 안 하면 옆으로 2 cm 넘게 비껴 보임 (pinky3 오프셋, 33 cm)
    assert abs(seen['tx']) > 0.02


def test_table_dock_uses_same_function():
    assert table_dock.cam_correct is md.cam_correct


def test_load_cam_file_reads_offset_and_resets(tmp_path, monkeypatch):
    cam = tmp_path / 'cam'
    cam.mkdir()
    f = cam / f'{socket.gethostname()}.json'
    f.write_text(json.dumps({'fx': 520.0, 'dy': 0.002, 'dyaw_deg': 2.5}))
    monkeypatch.setenv('FLEET_RECORDS_DIR', str(tmp_path))
    md.load_cam_file()
    assert md.FX == 520.0 and md.CAM_DY == 0.002 and abs(md.CAM_DYAW - math.radians(2.5)) < 1e-12
    f.write_text(json.dumps({'fx': 520.0}))          # 오프셋 없는 로봇(기준) -> 0으로 돌아감
    md.load_cam_file()
    assert md.CAM_DY == 0.0 and md.CAM_DYAW == 0.0


def test_measure_stable_passes_raw(monkeypatch):
    cam = object.__new__(md.Camera)
    calls = []

    def fake_measure(raw=False):
        calls.append(raw)
        return {'tz': 0.3, 'tx': 0.0 if raw else 0.01, 'yaw': 0.0}
    cam.measure = fake_measure
    assert cam.measure_stable(frames=3, raw=True)['tx'] == 0.0
    assert cam.measure_stable(frames=3)['tx'] == 0.01
    assert calls == [True] * 3 + [False] * 3

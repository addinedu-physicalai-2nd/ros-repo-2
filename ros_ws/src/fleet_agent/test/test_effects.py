"""안내 효과 (부저 · LED · LCD)를 가짜 장치로:  python3 -m pytest test/test_effects.py -v

연속 안내 때 두 번째부터 부저만 소리가 안 나던 문제:
같은 부저 객체로 buzzer_stop -> buzzer_start를 하면 PWM이 다시 안 켜지는 장치 (가짜 FakeBuzzer가 그렇게 흉내).
고친 뒤: PWM은 처음 한 번만 start, 소리 끄기는 duty 0만 -> 몇 번을 불러도 소리가 남
"""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

pytest.importorskip('PIL')


class FakeBuzzer:
    made = 0

    def __init__(self):
        FakeBuzzer.made += 1
        self.pwm = False          # PWM 켜짐
        self.dead = False         # stop 뒤 다시 start해도 안 켜지는 장치
        self.freq = 0
        self.duty = 0
        self.sounded = 0          # 실제로 소리가 난 음 수

    def buzzer_start(self):
        if self.dead:
            return                # 이 장치는 stop 뒤 다시 안 켜짐
        self.pwm = True

    def buzzer_stop(self):
        self.pwm = False
        self.dead = True

    def set_buzzer_freq(self, hz):
        self.freq = hz

    def set_buzzer_duty(self, d):
        self.duty = d
        if self.pwm and d > 0 and self.freq > 0:
            self.sounded += 1

    def clean(self):
        self.pwm = False


class FakeLED:
    def __init__(self):
        self.shows = 0

    def fill(self, c):
        self.c = c

    def show(self):
        self.shows += 1


class FakeLCD:
    def __init__(self):
        self.imgs = 0

    def set_backlight(self, v):
        pass

    def img_show(self, im):
        self.imgs += 1


@pytest.fixture
def fx(monkeypatch):
    pk = types.ModuleType('pinkylib')
    pk.LED, pk.Buzzer = FakeLED, FakeBuzzer
    lcd = types.ModuleType('pinky_lcd')
    lcd.LCD = FakeLCD
    monkeypatch.setitem(sys.modules, 'pinkylib', pk)
    monkeypatch.setitem(sys.modules, 'pinky_lcd', lcd)
    from fleet_agent import guide_effects as ge
    ge._HW.clear()
    ge._BUZZER_STARTED = False
    FakeBuzzer.made = 0
    return ge


def test_buzzer_sounds_every_time(fx):
    notes = sum(1 for hz, _ in fx.MELODY['welcome'] if hz)
    for i in range(3):                                   # 연속 안내 3번
        t = fx.play('welcome', log=lambda *_: None, hold=0.0)
        if t:
            t.join()
        b = fx._HW['buzzer']
        assert b.sounded == notes * (i + 1), f'{i + 1}번째 안내에서 부저 소리 안 남'
    assert FakeBuzzer.made == 1, '부저 객체는 하나만 (같은 핀에 PWM 겹치지 않게)'
    assert b.duty == 0, '끝나면 소리는 꺼져 있어야 함'


def test_led_and_lcd_still_work_every_time(fx):
    for _ in range(2):
        t = fx.play('arrived', log=lambda *_: None, hold=0.0)
        if t:
            t.join()
    assert fx._HW['led'].shows > 10 and fx._HW['lcd'].imgs > 4

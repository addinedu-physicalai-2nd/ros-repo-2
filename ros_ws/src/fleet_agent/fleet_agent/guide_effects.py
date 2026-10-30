"""안내 효과: LCD 웃는 얼굴 + LED + 짧은 부저 멜로디 (안내 담당자 코드 정리본)

play('welcome') / play('arrived'). 핑키 라이브러리가 없으면(시험 PC 등) 그냥 건너뛴다.
부저가 sleep으로 1초쯤 걸리므로 guide_node가 별도 스레드에서 부른다.
"""
import threading
import time

WELCOME = [(523, .16), (659, .16), (784, .16), (1047, .30)]
ARRIVED = [(784, .15), (1047, .18), (784, .12), (1047, .32)]


_LOCK = threading.Lock()      # 환영 · 도착 효과가 겹쳐 LED를 서로 켜고 끄지 않게 한 번에 하나


def _led_off(led, log=print):
    """LED 끄기 (끄는 show가 한 번 씹히는 경우가 있어 두 번 + 라이브러리에 정리 함수가 있으면 부름)"""
    for _ in range(2):
        try:
            led.fill((0, 0, 0))
            led.show()
        except Exception as e:  # noqa: BLE001
            log(f'LED 끄기 실패: {e}')
        time.sleep(0.05)
    for name in ('clear', 'close', 'clean'):
        f = getattr(led, name, None)
        if callable(f):
            try:
                f()
            except Exception:  # noqa: BLE001
                pass


def leds_off(log=print):
    """안내가 끝났을 때 (도착 · 손님 놓침 · 취소) LED를 확실히 끔"""
    try:
        from pinkylib import LED
    except Exception:  # noqa: BLE001
        return
    with _LOCK:
        try:
            _led_off(LED(), log)
        except Exception as e:  # noqa: BLE001
            log(f'LED 끄기 실패: {e}')


def play(kind, log=print):
    try:
        from pinkylib import LED, Buzzer
        from pinky_lcd import LCD
        from PIL import Image, ImageDraw
    except Exception as e:  # noqa: BLE001
        log(f'안내 효과 건너뜀 (라이브러리 없음: {e})')
        return
    with _LOCK:
        led = None
        try:
            lcd = LCD()
            image = Image.new('RGB', (320, 240), (255, 224, 80))
            d = ImageDraw.Draw(image)
            d.ellipse((55, 25, 265, 235), fill=(255, 201, 38), outline=(60, 50, 0), width=5)
            d.ellipse((105, 85, 125, 110), fill=(25, 25, 25))
            d.ellipse((195, 85, 215, 110), fill=(25, 25, 25))
            d.arc((105, 105, 215, 190), 10, 170, fill=(25, 25, 25), width=8)
            d.text((85, 8), 'WELCOME!' if kind == 'welcome' else 'ARRIVED!', fill=(25, 25, 25))
            lcd.set_backlight(255)
            lcd.img_show(image)

            led = LED()
            led.fill((255, 190, 0))
            led.show()
            b = Buzzer()
            b.buzzer_start()
            b.set_buzzer_duty(35)
            for hz, sec in (WELCOME if kind == 'welcome' else ARRIVED):
                b.set_buzzer_freq(hz)
                time.sleep(sec)
                b.set_buzzer_duty(0)
                time.sleep(.05)
                b.set_buzzer_duty(35)
            b.set_buzzer_duty(0)
            b.buzzer_stop()
            b.clean()
        except Exception as e:  # noqa: BLE001
            log(f'안내 효과 실패: {e}')
        finally:
            if led is not None:      # 부저에서 실패해도 LED는 끔 (예전: 실패하면 켜진 채로 남음)
                _led_off(led, log)

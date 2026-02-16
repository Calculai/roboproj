import time
import RPi.GPIO as GPIO
from gpiozero import LED

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

led = LED(23)

triggered = False
blinking = False
blink_start = 0
last_toggle = 0
LED_STATE = False

class Dist:
    def Measure(self, gp):
        GPIO_TRIGECHO = gp
        GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
        GPIO.output(GPIO_TRIGECHO, False)

        GPIO.output(GPIO_TRIGECHO, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGECHO, False)

        start = time.time()
        GPIO.setup(GPIO_TRIGECHO, GPIO.IN)

        while GPIO.input(GPIO_TRIGECHO) == 0:
            start = time.time()

        while GPIO.input(GPIO_TRIGECHO) == 1:
            stop = time.time()

        GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
        GPIO.output(GPIO_TRIGECHO, False)

        elapsed = stop - start
        return (elapsed * 34300) / 2.0


def update_blink():
    global blinking, LED_STATE, last_toggle

    if not blinking:
        return

    now = time.time()

    # stop blinking after 2 seconds
    if now - blink_start >= 2:
        blinking = False
        led.off()
        return

    # toggle LED every 0.25s
    if now - last_toggle >= 0.25:
        LED_STATE = not LED_STATE
        led.on() if LED_STATE else led.off()
        last_toggle = now


if __name__ == '__main__':
    sensor = Dist()

    while True:
        mes = sensor.Measure(18)
        print(mes)

        if 25 < mes < 30:
            if not triggered:
                triggered = True
                blinking = True
                blink_start = time.time()
                last_toggle = blink_start
        else:
            triggered = False

        update_blink()
        time.sleep(0.05)


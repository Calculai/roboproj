from gpiozero import LED
from time import sleep

led = LED(22)  # GPIO 22 (BCM numbering)

while True:
    led.on()
    sleep(1)
    led.off()
    sleep(1)

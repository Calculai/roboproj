from gpiozero import LED
from time import sleep

led = LED(17)  # GPIO 17 (BCM numbering)

while True:
    led.on()
    sleep(1)
    led.off()
    sleep(1)

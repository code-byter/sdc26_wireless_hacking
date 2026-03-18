# Hello World: Blink the onboard LED
from machine import Pin
import time

led = Pin(25, Pin.OUT)   # Pico onboard LED is GP25
print("Hello from Pico!")

while True:
    led.toggle()
    print("Blink!")
    time.sleep(0.5)
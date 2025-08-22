import pigpio
import smbus2
import numpy as np
import cv2
# removed when real test
try:
    import RPi.GPIO as GPIO
except ImportError:
    print("Running on non-RPi system. Using mock GPIO.")
    class GPIO:
        BCM = OUT = IN = LOW = HIGH = None
        @staticmethod
        def setmode(mode): pass
        @staticmethod
        def setup(pin, mode): pass
        @staticmethod
        def output(pin, state): pass
        @staticmethod
        def input(pin): return 0
        @staticmethod
        def cleanup(): pass
        
import time

def setup_controls(switch_pin, button_pin):
    GPIO.setup(switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


# Debounce time in seconds
DEBOUNCE_TIME = 0.2

def wait_for_power_on(switch_pin):
    print("Waiting for switch to turn ON...")
    while GPIO.input(switch_pin) == GPIO.LOW:
        time.sleep(0.1)
    print("Power ON. System is ready.")


def wait_for_button_press(button_pin):
    print("Press the button to start the robot...")
    while True:
        if GPIO.input(button_pin) == GPIO.HIGH:
            time.sleep(DEBOUNCE_TIME)  # Debounce
            if GPIO.input(button_pin) == GPIO.HIGH:
                print("Button pressed. Starting robot operation.")

def power_off():
    print("Robot will shut down")
    GPIO.cleanup()

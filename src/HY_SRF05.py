
import time

# Pin definitions (update based on your wiring)
# GPIO pin for Trigger
# GPIO pin for Echo
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


def setup_ultrasonic(TRIG_PIN, ECHO_PIN):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.1)  # let the sensor settle

def get_distance(TRIG_PIN, ECHO_PIN):
    # Send 10us pulse to trigger
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    # Wait for echo start
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    # Wait for echo end
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance_cm = pulse_duration * 17150  # Speed of sound = 34300 cm/s, divide by 2
    distance_cm = round(distance_cm, 2)

    return distance_cm


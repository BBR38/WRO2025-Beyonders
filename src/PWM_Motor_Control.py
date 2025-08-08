import pigpio
import time
import map

ESC_PIN = 18  # Must be a PWM-capable GPIO pin (like 18, 13, or 19)

pi = pigpio.pi()
if not pi.connected:
    exit("[ERROR] Could not connect to pigpio daemon")

def arm_esc(esc_pin):
    print("[ESC] Arming sequence...")
    pi.set_servo_pulsewidth(esc_pin, 0)
    time.sleep(1)
    pi.set_servo_pulsewidth(esc_pin, 2000)  # Max throttle
    time.sleep(2)
    pi.set_servo_pulsewidth(esc_pin, 1000)  # Min throttle
    time.sleep(2)
    print("[ESC] Armed and ready.")

def set_throttle(esc_pin, percent):
    percent = max(0, min(100, percent))
    pulse_width = 1000 + (percent / 100) * 1000  # 1000–2000 µs
    pi.set_servo_pulsewidth(esc_pin, pulse_width)
    print(f"[ESC] Throttle set to {percent}%")

def set_steering(servo_pin, angle):
    angle = max(0, min(180, angle))
    pulse_width = 1000 + (angle / 180.0) * 1000
    pi.set_servo_pulsewidth(servo_pin, pulse_width)
    print(f"[Servo] Angle {angle}° on pin {servo_pin}")

def stop_esc(esc_pin):
    pi.set_servo_pulsewidth(esc_pin, 0)
    pi.stop()
    print("[ESC] Stopped.")


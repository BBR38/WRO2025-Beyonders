import cv2
import numpy as np
import time
import math

import HY_SRF05
import Robot_Start_Finish
import map
import PWM_Motor_Control
import GY_521_MPU6050

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

# -----------------------------
# Globals & PID
# -----------------------------
heading_pid = GY_521_MPU6050.PID(Kp=2.0, Ki=0.0, Kd=0.1)
base_speed = 40
center_steering_angle = 90

orange_counter = 0
blue_counter = 0
orange_detected_prev = False
blue_detected_prev = False
prev_error = 0
prev_time = time.time()

# -----------------------------
# Initialization
# -----------------------------
def initalize():
    GPIO.setmode(GPIO.BCM)
    Robot_Start_Finish.setup_controls(map.gpio_map.get("Switch"), map.gpio_map.get("Button"))
    Robot_Start_Finish.wait_for_power_on(map.gpio_map.get("Switch"))
    Robot_Start_Finish.wait_for_button_press(map.gpio_map.get("Button"))
    for i in range(1, 5):
        HY_SRF05.setup_ultrasonic(map.gpio_map.get(f"ULTRASONIC_TRIG_{i}"),
                                  map.gpio_map.get(f"ULTRASONIC_ECHO_{i}"))
    GY_521_MPU6050.mpu6050_init()
    GY_521_MPU6050.calibrate_gyro()
    PWM_Motor_Control.arm_esc(map.gpio_map.get("Driver_PWM"))
    PWM_Motor_Control.set_steering(map.gpio_map["Servo_PWM"], 90)  # center
    print("[SYSTEM] Setup complete.")


def main():
    print(">> Robot is now running!")
    initalize()
    print("Starting robot movement...")    

    cap = cv2.VideoCapture(0)  # open camera

    try:
        while True:
            steer_angle, throttle, orange_count, blue_count = capture_and_process_frame(
                cap,
                base_speed=base_speed,
                distance_threshold=25,
                max_turn=30,
                center_steering_angle=center_steering_angle
            )

            # Exit by keyboard
            if steer_angle == "exit":
                print("[STOP] Exit requested.")
                break

            print(f"Steering: {steer_angle}°, Throttle: {throttle}%, "
                  f"Orange={orange_count}, Blue={blue_count}")

            # Stop condition: either counter reaches 12
            if orange_count >= 12 or blue_count >= 12:
                print("[STOP] Lap target reached.")
                break

            time.sleep(0.01)  # ~100Hz loop

    finally:
        stop_robot()
        cap.release()
        cv2.destroyAllWindows()

# -----------------------------
# Sensors
# -----------------------------
def read_ultrasonics():
    distances = {}
    for i in range(1, 5):
        d = HY_SRF05.get_distance(
            map.gpio_map[f"ULTRASONIC_TRIG_{i}"],
            map.gpio_map[f"ULTRASONIC_ECHO_{i}"]
        )
        distances[i-1] = d
    return distances

# -----------------------------
# Steering
# -----------------------------
def smooth_steering(error, center_angle=90, max_angle=30, Kp=1.0, Kd=0.5):
    """Compute smooth steering angle based on error (non-linear PD)."""
    global prev_error, prev_time
    now = time.time()
    dt = max(now - prev_time, 1e-6)
    derivative = (error - prev_error) / dt
    correction = Kp * math.tanh(error / 50) * max_angle + Kd * derivative
    steer_angle = int(max(60, min(120, center_angle + correction)))
    prev_error = error
    prev_time = now
    return steer_angle

def wall_following(target_distance=20):
    """Keep robot parallel to wall using US1 & US2."""
    distances = read_ultrasonics()
    left = distances[0]
    right = distances[2]
    error = (left - right) + ((left + right)/2 - target_distance)
    steer_angle = smooth_steering(error)
    return steer_angle

# -----------------------------
# Vision: Color + Obstacle
# -----------------------------
def color_and_obstacle_navigation(frame, front_distance, distance_threshold=25,
                                  max_turn=30, center_steering_angle=90, hsv=None):
    if hsv is None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red mask
    lower_red1 = np.array([0,100,100]); upper_red1 = np.array([10,255,255])
    lower_red2 = np.array([160,100,100]); upper_red2 = np.array([179,255,255])
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                              cv2.inRange(hsv, lower_red2, upper_red2))

    # Green mask
    lower_green = np.array([40,70,70]); upper_green = np.array([80,255,255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    red_pixels = cv2.countNonZero(mask_red)
    green_pixels = cv2.countNonZero(mask_green)

    error = 0
    if red_pixels > 500 and front_distance < distance_threshold:
        factor = math.tanh((distance_threshold - front_distance) / distance_threshold)
        error = -max_turn * factor
        print(f"🔴 Red detected ({front_distance}cm) → turn {error:.1f}°")
    elif green_pixels > 500 and front_distance < distance_threshold:
        factor = math.tanh((distance_threshold - front_distance) / distance_threshold)
        error = max_turn * factor
        print(f"🟢 Green detected ({front_distance}cm) → turn {error:.1f}°")
    elif front_distance < distance_threshold:
        factor = math.tanh((distance_threshold - front_distance) / distance_threshold)
        error = -max_turn * factor
        print(f"⚠️ Obstacle detected ({front_distance}cm) → turn {error:.1f}°")

    return smooth_steering(error, center_angle=center_steering_angle, max_angle=max_turn)

# -----------------------------
# Vision: Floor lines
# -----------------------------
def detect_floor_lines(frame, hsv=None,
                       orange_range=((5, 100, 100), (15, 255, 255)),
                       blue_range=((100, 150, 50), (140, 255, 255))):
    global orange_counter, blue_counter, orange_detected_prev, blue_detected_prev

    if hsv is None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_orange = cv2.inRange(hsv, np.array(orange_range[0]), np.array(orange_range[1]))
    mask_blue   = cv2.inRange(hsv, np.array(blue_range[0]), np.array(blue_range[1]))

    kernel = np.ones((3,3), np.uint8)
    mask_orange = cv2.dilate(mask_orange, kernel, iterations=1)
    mask_blue   = cv2.dilate(mask_blue, kernel, iterations=1)

    orange_detected, blue_detected = False, False

    if cv2.countNonZero(mask_orange) > 50:
        if not orange_detected_prev:
            orange_counter += 1
            print(f"🟧 Orange line detected! Count = {orange_counter}")
        orange_detected, orange_detected_prev = True, True
    else:
        orange_detected_prev = False

    if cv2.countNonZero(mask_blue) > 50:
        if not blue_detected_prev:
            blue_counter += 1
            print(f"🟦 Blue line detected! Count = {blue_counter}")
        blue_detected, blue_detected_prev = True, True
    else:
        blue_detected_prev = False

    return orange_detected, blue_detected

# -----------------------------
# Capture & Process
# -----------------------------
def capture_and_process_frame(cap, base_speed=30, distance_threshold=25,
                              max_turn=30, center_steering_angle=90,
                              heading_setpoint=0):
    """
    Capture a frame and process with priorities:
    1. Obstacle & red/green navigation
    2. Floor line detection (orange/blue)
    3. IMU heading correction
    Returns: steer_angle, throttle, orange_count, blue_count
    """
    global orange_counter, blue_counter

    ret, frame = cap.read()
    if not ret:
        return 90, base_speed, orange_counter, blue_counter

    # Preprocessing
    frame = cv2.resize(frame, (320, 240))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    steer_angle, throttle = center_steering_angle, base_speed

    # --- 1. Obstacle & red/green detection (highest priority) ---
    distances = read_ultrasonics()
    front_distance = distances[3]

    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0,100,100]), np.array([10,255,255])),
        cv2.inRange(hsv, np.array([160,100,100]), np.array([179,255,255]))
    )
    green_mask = cv2.inRange(hsv, np.array([40,70,70]), np.array([80,255,255]))
    red_green_active = (cv2.countNonZero(red_mask) > 500) or (cv2.countNonZero(green_mask) > 500)

    if red_green_active or front_distance < distance_threshold:
        steer_angle = color_and_obstacle_navigation(
            frame,
            front_distance,
            distance_threshold,
            max_turn,
            center_steering_angle,
            hsv=hsv
        )
        throttle = max(10, int(base_speed * min(1, front_distance / distance_threshold)))
        print(f"🚧 Obstacle/Color event → throttle={throttle}, steering={steer_angle}")
    
    else:
        # --- 2. Floor line detection ---
        orange_detected, blue_detected = detect_floor_lines(frame, hsv=hsv)
        if orange_detected or blue_detected:
            throttle = int(base_speed * 0.6)
            error = -max_turn if orange_detected else max_turn
            steer_angle = smooth_steering(error, center_angle=center_steering_angle, max_angle=max_turn)
            color = "🟧 Orange" if orange_detected else "🟦 Blue"
            print(f"{color} line → steering {error:+d}°")

        else:
            # --- 3. IMU heading correction (fallback) ---
            heading = GY_521_MPU6050.update_orientation()['z']
            correction = heading_pid.compute(setpoint=heading_setpoint, measured_value=heading)
            steer_angle = max(60, min(120, center_steering_angle + correction))
            print(f"🧭 IMU correction → Heading {heading:.2f}°, Steering {steer_angle:.1f}°")

    # --- 4. Apply motor commands ---
    PWM_Motor_Control.set_steering(map.gpio_map["Servo_PWM"], steer_angle)
    PWM_Motor_Control.set_throttle(PWM_Motor_Control.ESC_PIN, throttle)

    # --- 5. Debug display ---
    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return "exit", throttle, orange_counter, blue_counter

    return steer_angle, throttle, orange_counter, blue_counter

# -----------------------------
# Stop
# -----------------------------
def stop_robot():
    PWM_Motor_Control.stop_esc(PWM_Motor_Control.ESC_PIN)
    Robot_Start_Finish.power_off()
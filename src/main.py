import HY_SRF05
import Robot_Start_Finish
import smbus2
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

import time

import os
import sys

heading_pid = GY_521_MPU6050.PID(Kp=2.0, Ki=0.0, Kd=0.1)
base_speed = 40  # Base throttle percent (adjust for your robot)
center_steering_angle = 90  # Servo neutral position

def initalize():
    GPIO.setmode(GPIO.BCM)
    Robot_Start_Finish.setup_controls(map.gpio_map.get("Switch"), map.gpio_map.get("Button"))
    Robot_Start_Finish.wait_for_power_on(map.gpio_map.get("Switch"))
    Robot_Start_Finish.wait_for_button_press(map.gpio_map.get("Button"))
    for i in range(1, 5):
         HY_SRF05.setup_ultrasonic(map.gpio_map.get(f"ULTRASONIC_TRIG_{i}"), map.gpio_map.get(f"ULTRASONIC_ECHO_{i}"))
    GY_521_MPU6050.mpu6050_init()
    GY_521_MPU6050.calibrate_gyro()
    # Arm ESC before moving
    GY_521_MPU6050.arm_esc(map.gpio_map.get("Driver_PWM"))
    
    # PID for heading control
  #  global imu
  #  imu = mpu6050(0x68)
  #  imu_calibrate()
  #  set_speed(0)
  #  set_steering(90)
  #  print("[INIT] Complete.")


def main():
   print(">> Robot is now running!")
   initalize()
   print("Starting robot movement...")    
   while True:
        # Get current heading (yaw angle)
        heading = GY_521_MPU6050.update_orientation()['z']
        
        # Compute PID correction (try to keep heading = 0 degrees)
        correction = heading_pid.compute(setpoint=0, measured_value=heading)
        
        # Convert PID output to steering angle adjustment (-30 to +30 degrees)
        # Clamp final steering angle between 60 and 120 degrees (left/right limits)
        steer_angle = max(60, min(120, center_steering_angle + correction))
        
        # Apply throttle and steering signals
        GY_521_MPU6050.set_throttle(map.gpio_map.get("Servo_PWM"), base_speed)
        GY_521_MPU6050.set_steering(map.gpio_map.get("Driver_PWM"), steer_angle)
        
        print(f"Heading: {heading:.2f}°, Steering: {steer_angle:.1f}°, Throttle: {base_speed}%")
        
        time.sleep(0.05)  # 20 Hz control loop

import smbus2
import map
import time
import math
# ----------------------------
# MPU6050 Constants
# ----------------------------
GYRO_SCALE = 131.0 ,     # LSB/deg/sec for ±250°/s
ACCEL_SCALE = 16384.0     # LSB/g for ±2g
# ----------------------------
# Globals
# ----------------------------
bus = None
gyro_offset = {'x': 0, 'y': 0, 'z': 0}
gyro_angle = {'x': 0, 'y': 0, 'z': 0}
prev_time = time.time()

# ----------------------------
# Initialize MPU6050 Sensor
# ----------------------------
def mpu6050_init(bus_num=1):
    global bus
    bus = smbus2.SMBus(bus_num)
    bus.write_byte_data(map.address_map("MPU6050_ADDR"), map.address_map("PWR_MGMT_1"), 0)  # Wake up MPU6050
    time.sleep(0.1)

# ----------------------------
# Read a signed 16-bit word from MPU6050 register
# ----------------------------
def read_word_2c(reg):
    high = bus.read_byte_data(map.address_map("MPU6050_ADDR"),reg)
    low = bus.read_byte_data(map.address_map("MPU6050_ADDR"), reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    return val

# ----------------------------
# Raw Gyroscope Data (no scaling or offset)
# ----------------------------
def get_raw_gyro():
    x = read_word_2c(map.address_map("GYRO_XOUT_H"))
    y = read_word_2c(map.address_map("GYRO_YOUT_H"))
    z = read_word_2c(map.address_map("GYRO_ZOUT_H"))
    return {'x': x, 'y': y, 'z': z}

# ----------------------------
# Raw Accelerometer Data (no scaling)
# ----------------------------
def get_raw_accel():
    x = read_word_2c(map.address_map("ACCEL_XOUT_H"))
    y = read_word_2c(map.address_map("ACCEL_YOUT_H"))
    z = read_word_2c(map.address_map("ACCEL_ZOUT_H"))
    return {'x': x, 'y': y, 'z': z}

# ----------------------------
# Calibrate Gyro to remove static drift
# ----------------------------
def calibrate_gyro(samples=100):
    global gyro_offset
    print("Calibrating gyro, keep the sensor completely still...")
    sum_x = sum_y = sum_z = 0
    for _ in range(samples):
        g = get_raw_gyro()
        sum_x += g['x']
        sum_y += g['y']
        sum_z += g['z']
        time.sleep(0.01)  # 10 ms delay
    gyro_offset = {
        'x': sum_x / samples,
        'y': sum_y / samples,
        'z': sum_z / samples
    }
    print(f"Gyro calibrated: Offset = {gyro_offset}")

# ----------------------------
# Scaled Gyroscope Data in °/s
# ----------------------------
def get_gyro():
    raw = get_raw_gyro()
    return {
        'x': (raw['x'] - gyro_offset['x']) / GYRO_SCALE,
        'y': (raw['y'] - gyro_offset['y']) / GYRO_SCALE,
        'z': (raw['z'] - gyro_offset['z']) / GYRO_SCALE
    }

# ----------------------------
# Scaled Accelerometer Data in g
# ----------------------------
def get_accel():
    raw = get_raw_accel()
    return {
        'x': raw['x'] / ACCEL_SCALE,
        'y': raw['y'] / ACCEL_SCALE,
        'z': raw['z'] / ACCEL_SCALE
    }

# ----------------------------
# Track orientation angle by integrating gyro over time
# Returns cumulative angle in degrees
# ----------------------------
def update_orientation():
    global gyro_angle, prev_time
    now = time.time()
    dt = now - prev_time
    prev_time = now

    gyro = get_gyro()  # in deg/s

    gyro_angle['x'] += gyro['x'] * dt
    gyro_angle['y'] += gyro['y'] * dt
    gyro_angle['z'] += gyro['z'] * dt

    return gyro_angle

# ----------------------------
# PID Controller Class
# ----------------------------
class PID:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0
        self.prev_time = time.time()

    def compute(self, setpoint, measured_value):
        now = time.time()
        dt = now - self.prev_time
        if dt <= 0.0:
            dt = 1e-6  # Avoid division by zero

        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        self.prev_time = now
        return output

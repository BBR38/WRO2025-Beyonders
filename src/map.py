# gpio_constants.py or at the top of your script
gpio_map = {
    # Ultrasonic Sensor 1
    "ULTRASONIC_TRIG_1": 26,  # Physical pin 16
    "ULTRASONIC_ECHO_1": 24,  # Physical pin 18

    "ULTRASONIC_TRIG_2": 31,  # Physical pin 16
    "ULTRASONIC_ECHO_2": 21,  # Physical pin 18

    "ULTRASONIC_TRIG_3": 29,  # Physical pin 16
    "ULTRASONIC_ECHO_3": 19,  # Physical pin 18

    "ULTRASONIC_TRIG_4": 7,  # Physical pin 16
    "ULTRASONIC_ECHO_4": 23,  # Physical pin 18

    # Button and Power Switch
    "Button": 11,             # Physical pin 11
    "Switch": 36,             # Physical pin 13

    # Optional I2C (MPU6050)
    "I2C_SDA": 3,             # Physical pin 3
    "I2C_SCL": 5,             # Physical pin 5

    # Servo_Motor
    "Servo_PWM": 33,

    # Motor_Driver
    "Driver_PWM": 32,
   
}

address_map= {
    "MPU6050_ADDR" : 0x68,
    "PWR_MGMT_1"   : 0x6B,
    "GYRO_XOUT_H"  : 0x43,
    "GYRO_YOUT_H"  : 0x45,
    "GYRO_ZOUT_H"  : 0x47,
    "ACCEL_XOUT_H" : 0x3B,
    "ACCEL_YOUT_H" : 0x3D,
    "ACCEL_ZOUT_H" : 0x3F,
}
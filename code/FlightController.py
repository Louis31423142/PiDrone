import machine
import time
from machine import Pin, time_pulse_us
import math
import ibus
from mpu6050 import MPU6050

# Motor GPIO's 
gpio_motor1 = 6  # front left, clockwise
gpio_motor2 = 2  # front right, counterclockwise
gpio_motor3 = 4  # rear left, counterclockwise
gpio_motor4 = 0  # rear right, clockwise

# I2C pins used for MPU-6050
gpio_i2c_sda = 12
gpio_i2c_scl = 13

# Throttle settings
throttle_idle = 0                 
max_throttle = 1

# Desired Flight Controller Cycle frequency
target_cycle_hz = 250

# PID gains
pid_roll_kp = 0.00015
pid_roll_ki = 0.00075
pid_roll_kd = 0.0000075
pid_pitch_kp = pid_roll_kp
pid_pitch_ki = pid_roll_ki
pid_pitch_kd = pid_roll_kd
pid_yaw_kp = 0.001714287
pid_yaw_ki = 0.003428571
pid_yaw_kd = 0.0

# Max desired rates (degrees/second)
max_rate_roll = 30
max_rate_pitch = 30
max_rate_yaw = 50

# Wait a few seconds for the IMU to settle
print("Waiting 3 seconds for the IMU to settle...")
time.sleep(3)

# Overclock
machine.freq(250000000)
print("Overclocked to 250,000,000")

#setup rx
rc_uart = 1
rc:ibus.IBus = ibus.IBus(rc_uart)
print("RC receiver set up")

# Set up IMU (MPU-6050)
i2c = machine.I2C(0, sda=machine.Pin(gpio_i2c_sda), scl=machine.Pin(gpio_i2c_scl))
mpu6050_address = 0x68
i2c.writeto_mem(mpu6050_address, 0x6B, bytes([0x01]))  # Wake it up
i2c.writeto_mem(mpu6050_address, 0x1A, bytes([0x05]))  # Set low pass filter to 5 (0-6)
i2c.writeto_mem(mpu6050_address, 0x1B, bytes([0x08]))  # Set gyro scale to 1 (0-3)

is_armed = False

# Function to normalise throttle inputs
def normalize(value, original_min, original_max, new_min, new_max):
    return new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))

# Function to check the arm/disarm status based on CH5 input
def check_arming():
    global is_armed
    rc_data = rc.read()
    pulse_width_arm = rc_data[5]
    if pulse_width_arm == 2000:
        is_armed = True
    elif pulse_width_arm == 1000:
        is_armed = False

# Define the translate_pair function
def translate_pair(high_byte, low_byte):
    value = (high_byte << 8) | low_byte
    if value > 32767:
        value -= 65536
    return value
  
# Function to convert throttle value to PWM duty cycle
def calculate_duty_cycle(throttle_value):
    min_pulse_width_us = 1000  
    max_pulse_width_us = 2000  
    throttle_value = max(0.0, min(throttle_value, 1.0))
    pulse_width_us = min_pulse_width_us + (max_pulse_width_us - min_pulse_width_us) * throttle_value
    duty_cycle = int((pulse_width_us / 20000) * 65535)
    duty_cycle = max(0, min(duty_cycle, 65535)) 
    return duty_cycle

# ESC arming sequence
def arm_escs():
    duty_cycle = calculate_duty_cycle(0)
    M1.duty_u16(duty_cycle)
    M2.duty_u16(duty_cycle)
    M3.duty_u16(duty_cycle)
    M4.duty_u16(duty_cycle)
    print(f"Arming ESCs with minimum throttle, duty cycle: {duty_cycle}")
    time.sleep(5)

# Measure gyro bias
gxs, gys, gzs = [], [], []
started_at_ticks_ms = time.ticks_ms()
while ((time.ticks_ms() - started_at_ticks_ms) / 1000) < 3.0:
    gyro_data = i2c.readfrom_mem(mpu6050_address, 0x43, 6)  # Read 6 bytes (2 for each axis)
    gyro_x = translate_pair(gyro_data[0], gyro_data[1]) / 65.5
    gyro_y = translate_pair(gyro_data[2], gyro_data[3]) / 65.5
    gyro_z = translate_pair(gyro_data[4], gyro_data[5]) / 65.5 * -1
    gxs.append(gyro_x)
    gys.append(gyro_y)
    gzs.append(gyro_z)
    time.sleep(0.025)
gyro_bias_x = sum(gxs) / len(gxs)
gyro_bias_y = sum(gys) / len(gys)
gyro_bias_z = sum(gzs) / len(gzs)
print("GYRO BIAS MEASURED")

# Set up PWM's for the motors
freq = 50
M1 = machine.PWM(machine.Pin(gpio_motor1))
M2 = machine.PWM(machine.Pin(gpio_motor2))
M3 = machine.PWM(machine.Pin(gpio_motor3))
M4 = machine.PWM(machine.Pin(gpio_motor4))
M1.freq(freq) 
M2.freq(freq)
M3.freq(freq)
M4.freq(freq)
print("MOTORS PWM SET AT 50HZ")

# Constants calculations
cycle_time_seconds = 1.0 / target_cycle_hz
cycle_time_us = int(round(cycle_time_seconds * 1000000, 0))
throttle_range = max_throttle - throttle_idle
i_limit = 150.0

# State variables - PID related
roll_last_integral = 0.0
roll_last_error = 0.0
pitch_last_integral = 0.0
pitch_last_error = 0.0
yaw_last_integral = 0.0
yaw_last_error = 0.0

arm_escs()
print("ESC ARMED")

print("PREPARE FOR TAKE OFF")

while True:
    check_arming()
    
    if is_armed:
        # Mark start time
        loop_begin_us = time.ticks_us()

        # Capture and process IMU data
        gyro_data = i2c.readfrom_mem(mpu6050_address, 0x43, 6) 
        gyro_x = ((translate_pair(gyro_data[0], gyro_data[1]) / 65.5) - gyro_bias_x)
        gyro_y = ((translate_pair(gyro_data[2], gyro_data[3]) / 65.5) - gyro_bias_y) * -1
        gyro_z = ((translate_pair(gyro_data[4], gyro_data[5]) / 65.5) * -1) - gyro_bias_z

        #gather and normalise inputs
        rc_data = rc.read()
        desired_throttle = normalize(rc_data[3], 1000.0, 2000.0, 0.0, 1.0) 
        desired_pitch_rate = max_rate_pitch*(normalize(rc_data[2], 1000.0, 2000.0, -1.0, 1.0)) * -1 
        desired_roll_rate = max_rate_roll*normalize(rc_data[1], 1000.0, 2000.0, -1.0, 1.0)
        desired_yaw_rate = max_rate_yaw*normalize(rc_data[4], 1000.0, 2000.0, -1.0, 1.0)

        # Calculate errors 
        error_roll = desired_roll_rate - gyro_x
        error_pitch = desired_pitch_rate - gyro_y
        error_yaw = desired_yaw_rate - gyro_z
        print(error_roll)

        # Roll PID calc
        roll_p = error_roll * pid_roll_kp
        roll_i = roll_last_integral + (error_roll * pid_roll_ki * cycle_time_seconds)
        roll_i = max(min(roll_i, i_limit), -i_limit)
        roll_d = pid_roll_kd * (error_roll - roll_last_error) / cycle_time_seconds
        pid_roll = roll_p + roll_i + roll_d

        # Pitch PID calc
        pitch_p = error_pitch * pid_pitch_kp
        pitch_i = pitch_last_integral + (error_pitch * pid_pitch_ki * cycle_time_seconds)
        pitch_i = max(min(pitch_i, i_limit), -i_limit)
        pitch_d = pid_pitch_kd * (error_pitch - pitch_last_error) / cycle_time_seconds
        pid_pitch = pitch_p + pitch_i + pitch_d

        # Yaw PID calc
        yaw_p = error_yaw * pid_yaw_kp
        yaw_i = yaw_last_integral + (error_yaw * pid_yaw_ki * cycle_time_seconds)
        yaw_i = max(min(yaw_i, i_limit), -i_limit)
        yaw_d = pid_yaw_kd * (error_yaw - yaw_last_error) / cycle_time_seconds
        pid_yaw = yaw_p + yaw_i + yaw_d

        # Throttle mixing
        t1 = desired_throttle + pid_pitch + pid_roll - pid_yaw
        t2 = desired_throttle + pid_pitch - pid_roll + pid_yaw
        t3 = desired_throttle - pid_pitch + pid_roll + pid_yaw
        t4 = desired_throttle - pid_pitch - pid_roll - pid_yaw

        # Clamp throttle values to ensure they are within 0 to 1 range
        t1 = max(0.0, min(t1, 1.0))
        t2 = max(0.0, min(t2, 1.0))
        t3 = max(0.0, min(t3, 1.0))
        t4 = max(0.0, min(t4, 1.0))

        # Convert throttle values to PWM duty cycles
        M1.duty_u16(calculate_duty_cycle(t1))
        M2.duty_u16(calculate_duty_cycle(t2))
        M3.duty_u16(calculate_duty_cycle(t3))
        M4.duty_u16(calculate_duty_cycle(t4))

        # Save state values for the next loop
        roll_last_error = error_roll
        pitch_last_error = error_pitch
        yaw_last_error = error_yaw
        roll_last_integral = roll_i
        pitch_last_integral = pitch_i
        yaw_last_integral = yaw_i

        # Maintain regular loop time 
        loop_end_us = time.ticks_us()
        elapsed_us = time.ticks_diff(loop_end_us, loop_begin_us)
        if elapsed_us < cycle_time_us:
            time.sleep_us(cycle_time_us - elapsed_us)
    
    else:
        # Disarmed
        M1.duty_u16(calculate_duty_cycle(0))
        M2.duty_u16(calculate_duty_cycle(0))
        M3.duty_u16(calculate_duty_cycle(0))
        M4.duty_u16(calculate_duty_cycle(0))
        roll_last_integral = 0.0
        roll_last_error = 0.0
        pitch_last_integral = 0.0
        pitch_last_error = 0.0
        yaw_last_integral = 0.0
        yaw_last_error = 0.0








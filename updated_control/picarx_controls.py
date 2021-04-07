try:
    from ezblock import *
except ImportError:
    print(" This computer does not appear to be a PiCar -X system(/ opt/ ezblock is not present ). Shadowing hardware "
           "calls with substitute functions ")
    from sim_ezblock import *
import time
import atexit
import numpy as np
from logdecorator import log_on_start, log_on_end, log_on_error
import logging

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="% H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


class MotorControl:

    def __init__(self):
        PERIOD = 4095
        PRESCALER = 10
        TIMEOUT = 0.02

        self.dir_servo_pin = Servo(PWM('P2'))
        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")

        self.drive_angle = 0  # current turning angle

        self.Servo_dir_flag = 1
        self.dir_cal_value = 5

        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.motor_direction_pins, self.right_rear_pwm_pin]
        self.cali_dir_value = [1, -1]
        self.cali_speed_value = [0, 0]

        for pin in self.motor_speed_pins:
            pin.period(PERIOD)
            pin.prescaler(PRESCALER)

    def set_motor_speed(self, motor, speed):
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        if speed != 0:
            # speed = int(speed /2 ) + 50
            speed = speed
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def set_dir_servo_angle(self, value):
        self.drive_angle = value
        self.dir_servo_pin.angle(value + self.dir_cal_value)

    def forward(self, speed):
        sign = self.angle / abs(self.angle)
        dist_arc = -0.095 / np.cos(sign * ((abs(self.angle) + 90) * np.pi / 180))
        inside_wheel_speed = ((dist_arc - 0.06) * speed) / dist_arc
        outside_wheel_speed = ((dist_arc + 0.06) * speed) / dist_arc
        if self.drive_angle < 0:  # turning left (ccw)
            self.set_motor_speed(1, -1 * inside_wheel_speed)  # Left wheel inside
            self.set_motor_speed(2, -1 * outside_wheel_speed)  # Right wheel outside
        else:  # turning right (cw)
            self.set_motor_speed(1, -1 * outside_wheel_speed)  # left wheel outside
            self.set_motor_speed(2, -1 * inside_wheel_speed)  # right wheel inside

    def backwards(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)




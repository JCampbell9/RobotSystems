try:
    from ezblock import *
    from ezblock import __reset_mcu__
    __reset_mcu__()
    time.sleep(0.01)
except ImportError:
    print(" This computer does not appear to be a PiCar -X system(/ opt/ ezblock is not present ). Shadowing hardware "
           "calls with substitute functions ")
    from sim_ezblock import *
import time
import atexit
import numpy as np
from logdecorator import log_on_start, log_on_end, log_on_error
import logging


##############################################################################################
# Author: Josh Campbell,
# Date: 4/16/2021
# control classes for picarx
##############################################################################################
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="% H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


class MotorControl:
    """
    Motor control class for the picar-x
    """

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
        self.speed = 0

        self.Servo_dir_flag = 1
        self.dir_cal_value = 5

        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        self.cali_dir_value = [1, -1]
        self.cali_speed_value = [0, 0]

        for pin in self.motor_speed_pins:
            pin.period(PERIOD)
            pin.prescaler(PRESCALER)

    def set_motor_speed(self, motor, speed):
        """
        Sets the speed for a specified motor
        :param motor: 1 left motor, 2 is right motor
        :param speed: pwm value to set for speed
        :return: none
        """
        motor -= 1  # from old code
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
        """
        Steers the car, positive is right turn negative is left turn
        :param value: degrees to turn the wheel
        :return: none
        """
        self.drive_angle = value
        self.dir_servo_pin.angle(value + self.dir_cal_value)
        self.differential_speed()

    def forward(self, speed):
        """
        Move the car forward
        :param speed: pwm value for the speed
        :return: none
        """
        self.speed = speed
        self.differential_speed()

    def differential_speed(self):
        try:
            sign = self.drive_angle / abs(self.drive_angle)  # gets if the angle is positive or negative
        except ZeroDivisionError:
            sign = 1
        dist_arc = 0.095 * np.tan((90 - abs(self.drive_angle)) * np.pi / 180)
        inside_wheel_speed = ((dist_arc - 0.06) * self.speed) / dist_arc
        outside_wheel_speed = ((dist_arc + 0.06) * self.speed) / dist_arc
        if sign < 0:  # turning left (ccw), negative
            self.set_motor_speed(1, -1 * inside_wheel_speed)  # Left wheel inside
            self.set_motor_speed(2, -1 * outside_wheel_speed)  # Right wheel outside
        else:  # turning right (cw), positive
            self.set_motor_speed(1, -1 * outside_wheel_speed)  # left wheel outside
            self.set_motor_speed(2, -1 * inside_wheel_speed)  # right wheel inside


    def backwards(self, speed):
        """
        Move the car backwards
        :param speed:
        :return:
        """
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def set_power(self, speed):
        """
        Set power
        :param speed:
        :return:
        """
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)


class SensorControl:

    def __init__(self):
        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')
        self.adc_values = []

    def get_adc_values(self):
        self.adc_values = [self.S0.read(), self.S1.read(), self.S2.read()]
        return self.adc_values


class Interpreter:

    def __init__(self, sensitivity=50, polarity=-1):

        # polarity -1 is the line is lighter, 1 the line is darker
        # sensitivity the needed delta between the sensor reading to determine an edge
        self.sensitivity = sensitivity
        self.polarity = polarity

    def robot_relative_to_line(self, adc_values=[0, 0, 0]):
        left_middle = adc_values[0] - adc_values[1]
        right_middle = adc_values[2] - adc_values[1]

        # Assuming one of the sensors is over the line at all times

        # Written with the assumption of dark line on a light background
        # -1 indicates that the right, of the two, sensor is over the line
        # 1 indicates the left of the two sensors is over the line
        # 0 indicates that sensor sees the same as the middle sensor
        if left_middle > self.sensitivity:
            l_m = -1  # middle sensor over line
        elif left_middle < (-1 * self.sensitivity):
            l_m = 1  # left sensor over line
        else:
            l_m = 0  # either both sensors are over the line or not

        if right_middle > self.sensitivity:
            r_m = 1  # middle sensor over the line
        elif right_middle < (-1 * self.sensitivity):
            r_m = -1  # right sensor over the line
        else:
            r_m = 0  # either both sensors are over the line or not

        # this flips signs if needed for lighter lines on darker backgrounds
        r_m *= self.polarity
        l_m *= self.polarity

        # these conditions are when only the left or right sensor picks up the lines
        # versus the middle and the left/right
        if l_m == 0 and r_m == -1:
            r_m *= 2
        elif l_m == 1 and r_m == 0:
            l_m *= 2

        # robot location relative to the line
        # 1: two sensors to the left of the line
        # -1: two sensors to the left of the line
        robo_loc = (r_m + l_m) / 2
        return robo_loc


class Controller:

    def __init__(self, scale_factor=20):

        self.scale_factor = scale_factor
        self.line_detector = SensorControl()
        self.line_interpreter = Interpreter()
        self.motor_controller = MotorControl()

    def line_follower(self):

        offset = self.line_interpreter.robot_relative_to_line(self.line_detector.get_adc_values())
        steering_angle = self.scale_factor * (1 * offset)
        self.motor_controller.set_dir_servo_angle(steering_angle)
        return steering_angle


class CameraControl:

    def __init__(self):
        pass


if __name__ == "__main__":
    steer = Controller(20)
    # steer.motor_controller.forward(50)
    while True:
        steer.line_follower()
        time.sleep(.1)

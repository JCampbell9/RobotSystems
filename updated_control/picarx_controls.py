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
import cv2
import math



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
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))

        self.Servo_dir_flag = 1
        self.cam_cal_value_1 = 0
        self.cam_cal_value_2 = 0

    def camera_x_axis(self, value):
        self.camera_servo_pin1.angle(-1 * (value + self.cam_cal_value_1))

    def camera_y_axis(self, value):
        self.camera_servo_pin2.angle(-1 * (value + self.cam_cal_value_2))


class CameraSensor:
    """
    This class is based off of David Tian's guide found here: https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
    """
    def __init__(self):

        self.lower_blue = np.array([60, 0, 0])
        self.upper_blue = np.array([150, 255, 255])

    def steering_angle(self, image):
        self.image = cv2.imread(image)
        edge_mask = self.edge_detector()
        crop_image = self.limit_image(edge_mask)
        line_seg = self.line_seg_detector(crop_image)

        average_line = self.line_joiner(line_seg)
        # cv2.imshow("line_seg", line_segments)
        center_line = self.make_points(self.image, average_line)
        # line_img = self.display_lines(frame=self.image, lines=[center_line])
        dir_angle = self.direction_angle(self.image, center_line)
        return dir_angle

    def edge_detector(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        edges = cv2.Canny(mask, 200, 400)
        return edges

    def limit_image(self, edge_img):
        height, width = edge_img.shape
        mask = np.zeros_like(edge_img)

        polygon = np.array([[(0, height*.5), (width, height*.5), (width, height), (0, height)]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        crop_img = cv2.bitwise_and(edge_img, mask)
        return crop_img

    def line_seg_detector(self, frame):
        rho = 1
        angle = np.pi / 180
        min_threshold = 10
        line_segments = cv2.HoughLinesP(frame, rho, angle, min_threshold, np.array([]), minLineLength=10, maxLineGap=5)
        return line_segments
        # print(line_segments)

    def line_joiner(self, lines):
        line_list = []
        print(lines[0])
        for line_seg in lines:
            for x1, y1, x2, y2 in line_seg:
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                line_list.append((fit[0], fit[1]))

        # print(line_list)
        av = np.average(line_list, axis=0)

        # print(av)
        return av

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def direction_angle(self, frame, av_line):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        x1, _, x2, _ = av_line[0]
        print(av_line)
        offset_x = x2 - width/2
        offset_y = int(height/2)
        print(offset_x)

        rad_angle = math.atan2(offset_y, offset_x)
        # print(f'radiant angle = {rad_angle}')

        deg_ang = 90 - int(rad_angle * 180 / np.pi)

        return deg_ang

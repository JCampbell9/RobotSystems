import time
#import sys
#sys.path.append(r'/opt/ezblock')
#from ezblock import __reset_mcu__
#__reset_mcu__()
#time.sleep(0.01)

from picarx_improved import *

def movement_a():
    set_dir_servo_angle(0)
    forward(speed=60)
    time.sleep(3)
    stop()
    time.sleep(.5)
    backward(60)
    time.sleep(5)
    stop()
    time.sleep(.5)


def movement_b_left():
    set_dir_servo_angle(-40)
    backward(50)
    time.sleep(.5)
    set_dir_servo_angle(40)
    # backward(60)
    time.sleep(.5)
    set_dir_servo_angle(0)
    time.sleep(.25)
    stop()
    time.sleep(.2)


def movement_b_right():
    set_dir_servo_angle(40)
    backward(50)
    time.sleep(.5)
    set_dir_servo_angle(-40)
    time.sleep(.5)
    set_dir_servo_angle(0)
    time.sleep(.25)
    stop()
    time.sleep(.25)


def movement_c():
    """
    Three point turn
    :return:
    """

    set_dir_servo_angle(-30)
    forward(60)
    time.sleep(.75)
    #set_dir_servo_angle(0)
   # forward(60, 0)
    time.sleep(.5)
    stop()
    time.sleep(.5)
    set_dir_servo_angle(40)
    backward(60)
    time.sleep(.75)
    set_dir_servo_angle(0)
    time.sleep(.5)
    stop()
    time.sleep(.5)
    set_dir_servo_angle(0)
    forward(60)
    time.sleep(.5)
    set_dir_servo_angle(0)
    forward(60)
    time.sleep(.5)
    stop()
    time.sleep(.5)


if __name__ == '__main__':

    while True:
        action = input("what manuever?(a, b_l, b_r, c)  ")

        if action == 'a':
            movement_a()
        elif action == 'b_l':
            movement_b_left()
        elif action == 'b_r':
            movement_b_right()
        elif action == 'c':
            movement_c()
        elif action == 'x':
            break

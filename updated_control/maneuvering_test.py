from picarx_improved import *
import time


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
    set_dir_servo_angle(30)
    backward(60)
    time.sleep(.5)
    set_dir_servo_angle(0)
    backward(60)
    time.sleep(.5)
    stop()
    time.sleep(.5)

def movement_c():
    """
    Three point turn
    :return:
    """

    set_dir_servo_angle(-30)
    forward(60, -30)
    time.sleep(.5)
    set_dir_servo_angle(0)
    forward(60, 0)
    time.sleep(.5)
    stop()
    time.sleep(.5)
    set_dir_servo_angle(30)
    backward(60)
    time.sleep(.5)
    set_dir_servo_angle(0)
    time.sleep(.5)
    stop()
    time.sleep(.5)
    forward(60)
    time.sleep(1)
    stop()
    time.sleep(.5)



if __name__=='__main__':

    while True:
        action = input("what manuever?(a, b_l, b_r, c)  ")

        if action == 'a':
            movement_a()
        elif action == 'b_l':
            movement_b_left()
        elif action == 'c':
            movement_c()
        elif action == 'x':
            break

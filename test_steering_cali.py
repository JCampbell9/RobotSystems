from picarx_improved import *
import time

set_dir_servo_angle(0)

forward(50)

print("going forward...")

time.sleep(5)

print("stopping...")
stop()
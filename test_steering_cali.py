from picarx_improved import *
import time

set_dir_servo_angle(18)
time.sleep(.5)

forward(60)
print("going forward...")

time.sleep(3)

print("stopping...")
stop()

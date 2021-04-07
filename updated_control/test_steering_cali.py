from picarx_improved import *
import time

set_dir_servo_angle(0) 
time.sleep(1)

forward(60)
print("going forward...")

time.sleep(3)

print("stopping...")
stop()

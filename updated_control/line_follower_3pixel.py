from picarx_controls import *


if __name__ == "__main__":
    steer = Controller(20)
    # steer.motor_controller.forward(50)
    while True:
        steer.line_follower()
        time.sleep(.1)

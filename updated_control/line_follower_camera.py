from picarx_controls import *
from ezblock import camera


if __name__ == "__main__":
    line_detect = CameraSensor()
    steer = MotorControl()
    steer.forward(50)
    cam = camera.Camera()
    image = 'camera_image.jpg'

    cam_direction = CameraControl()
    cam_direction.camera_x_axis(0)
    cam_direction.camera_y_axis(0)

    while True:

        cam.camera.capture(image)
        steering_angle = line_detect.steering_angle(image)
        steer.set_dir_servo_angle(steering_angle)
        print(steering_angle)
        time.sleep(.1)


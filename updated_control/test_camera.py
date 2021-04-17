from ezblock import camera
from picamera.array import PiRGBArray
import cv2

cam = camera.Camera()
rawCapture = PiRGBArray(cam.camera)
cam.camera.capture(rawCapture, format="bgr")
image = rawCapture.array

cv2.imwrite('test_image.jpg',image)

#cv2.imshow("Image", image)

cam.start()


def forever():
    pass

if __name__ == "__main__":

    while True:
        forever()

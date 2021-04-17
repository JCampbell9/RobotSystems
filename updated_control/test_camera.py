from ezblock import camera
from picamera.array import PiRGBArray
import cv2
import numpy as np

cam = camera.Camera()
rawCapture = PiRGBArray(cam.camera)
cam.camera.capture(rawCapture, format="bgr")
image = rawCapture.array

cam.camera.capture('base_image_test.jpg')

cv2.imwrite('test_image.jpg', image)
#frame = cv2.imread(image)

#cv2.imwrite('test2_image.jpg', frame)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower_blue = np.array([60, 40, 40])
upper_blue = np.array([150, 255, 255])
mask = cv2.inRange(hsv, lower_blue, upper_blue)
cv2.imwrite('testing_image2.jpg', mask)

#cv2.imshow("Image", image)

cam.start()


def forever():
    pass

if __name__ == "__main__":

    while True:
        forever()

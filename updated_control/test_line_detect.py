import cv2
import numpy as np
import time


image = cv2.imread('test_image.jpg')

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_blue = np.array([60, 0, 40])
upper_blue = np.array([150, 255, 255])
mask = cv2.inRange(hsv, lower_blue, upper_blue)


cv2.imshow("Image", mask)

cv2.waitKey(0)


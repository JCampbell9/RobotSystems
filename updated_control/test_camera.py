from ezblock import camera
from picamera.array import PiRGBArray
import cv2

cam = camera.Camera()
rawCapture = PiRGBArray(cam.camera)
cam.camera.capture(rawCapture, format="bgr")
image = rawCapture.array

cv2.imwrite('test_image.jpg', image)
frame = cv2.imread(image)

cv2.imwrite('test2_image.jpg', frame)

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


#cv2.imshow("Image", image)

cam.start()


def forever():
    pass

if __name__ == "__main__":

    while True:
        forever()

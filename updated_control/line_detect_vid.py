import cv2
import numpy as np
import time

# modified from
#  https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_trackbar/py_trackbar.html

def nothing(x):
    pass


def main():
    cap = cv2.VideoCapture('test_line.mp4')
    # cap = cv2.imread('test_image.jpg')
    cv2.namedWindow('image')
    cv2.namedWindow('mask')
    lower = np.array([60, 0, 40])      # init to blue
    upper = np.array([150, 255, 255])
    # Make trackbars, with default values above
    cv2.createTrackbar('H_lower','image',lower[0],255,nothing)
    cv2.createTrackbar('H_upper','image',upper[0],255,nothing)
    cv2.createTrackbar('S_lower','image',lower[1],255,nothing)
    cv2.createTrackbar('S_upper','image',upper[1],255,nothing)
    cv2.createTrackbar('V_lower','image',lower[2],255,nothing)
    cv2.createTrackbar('V_upper','image',upper[2],255,nothing)
    print("Showing video. With the CV window in focus, press q to exit, p to pause.")
    while(1):
        ret, img = cap.read()
        if not ret: break
        # get current positions of four trackbars
        lower[0] = cv2.getTrackbarPos('H_lower', 'image')
        lower[1] = cv2.getTrackbarPos('S_lower', 'image')
        lower[2] = cv2.getTrackbarPos('V_lower', 'image')
        upper[0] = cv2.getTrackbarPos('H_upper', 'image')
        upper[1] = cv2.getTrackbarPos('S_upper', 'image')
        upper[2] = cv2.getTrackbarPos('V_upper', 'image')
        # Create mask by thresholding HSV image
        mask = detect_color(img, lower, upper)
        cv2.imshow('image', img)
        cv2.imshow('mask', mask)
        key = cv2.waitKey(66)   # Delay for 66 ms
        if key == ord('q'): # Press q to exit, p to pause
            break
        if key == ord('p'):
            cv2.waitKey(-1) #wait until any key is pressed
    cv2.destroyAllWindows()


def detect_color(frame, lower, upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    return mask


if __name__=="__main__":
    main()
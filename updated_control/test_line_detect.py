import cv2
import numpy as np
import time
import math


class LineDetector:

    def __init__(self, image):
        self.image = cv2.imread(image)

        self.lower_blue = np.array([60, 0, 0])
        self.upper_blue = np.array([150, 255, 255])
        self.edge_detector()

    def edge_detector(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        edges = cv2.Canny(mask, 200, 400)

        # cv2.imshow("image_mask", mask)
        # cv2.imshow("Image2", edges)
        crop_img = self.limit_image(edges)
        cv2.imshow("cropped", crop_img)
        self.line_seg_detector(crop_img)

    def limit_image(self, edge_img):
        height, width = edge_img.shape
        mask = np.zeros_like(edge_img)

        polygon = np.array([[(0, height*.5), (width, height*.5), (width, height), (0, height)]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        crop_img = cv2.bitwise_and(edge_img, mask)
        return crop_img

    def line_seg_detector(self, edge_img):
        rho = 1
        angle = np.pi / 180
        min_threshold = 10
        line_segments = cv2.HoughLinesP(edge_img, rho, angle, min_threshold, np.array([]), minLineLength=10, maxLineGap=5)
        print(line_segments)
        average_line = self.slope(line_segments)
        # cv2.imshow("line_seg", line_segments)
        center_line = self.make_points(self.image, average_line)
        line_img = self.display_lines(frame=self.image, lines=[center_line])
        self.steering_angle(self.image, center_line)

        cv2.imshow('lines', line_img)
        cv2.waitKey(0)


    def slope(self, lines):
        line_list = []
        print(lines[0])
        for line_seg in lines:
            for x1, y1, x2, y2 in line_seg:
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                line_list.append((fit[0], fit[1]))

        print(line_list)
        av = np.average(line_list, axis=0)

        print(av)
        return av

    def steering_angle(self, frame, av_line):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        x1, _, x2, _ = av_line[0]
        print(av_line)
        offset_x = x2 - width/2
        offset_y = int(height/2)
        print(offset_x)

        rad_angle = math.atan2(offset_y, offset_x)
        print(f'radiant angle = {rad_angle}')

        deg_ang = 90 - int(rad_angle * 180 / np.pi)
        print(deg_ang)
        cv2.line(heading_image, (int(width / 2), int(height)), (int((width / 2) + height / 2 / math.tan(rad_angle)), 0), (0, 0, 255), 10)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
        cv2.imshow('lines2', heading_image)



    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=10):
        height, width, _ = frame.shape
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

        cv2.line(line_image, (int(width/2), 0), (int(width/2), int(height)), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

    # def combine_lines(self, lines):
    #     line_list = []
    #     print(lines[0])
    #     for line_seg in lines:
    #         for x1, y1, x2, y2 in line_seg:
    #             fit = np.polyfit((x1, x2), (y1, y2), 1)
    #             line_list.append((fit[0], fit[1]))

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]


if __name__ == '__main__':
    LineDetector('base_image_test.jpg')
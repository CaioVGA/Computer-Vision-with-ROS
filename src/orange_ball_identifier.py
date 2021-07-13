#!/usr/bin/env python

# Author: Caio Vinicius Gomes Araujo
#
# Software made to stream video via webcam in real time,
# doing a segmentation of an orange ball. 

# Libraries imports
import numpy as np
import cv2
from imutils.video import VideoStream, FPS
import time


def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    if show:
        cv2.imshow("RGB Image", rgb_image)
    return rgb_image


def filter_color(image, lower_bound_color, upper_bound_color):
    # convert the image into the HSV color space

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image", hsv_image)

    # find the upper and lower bounds of the yellow color (tennis ball)
    #yellowLower = (30, 150, 100)
    #yellowUpper = (60, 255, 255)

    # define a mask using the lower and upper bounds of the yellow color
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask


def convert_rgb_to_gray(rgb_image, show):
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    if show:
        cv2.imshow("Gray Image", gray_image)
    return gray_image


def convert_gray_to_binary(gray_image, adaptive, show):
    if adaptive:
        binary_image = cv2.adaptiveThreshold(gray_image,
                                             255,
                                             cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv2.THRESH_BINARY_INV, 5, 2)
    else:
        _, binary_image = cv2.threshold(
            gray_image, 127, 255, cv2.THRESH_BINARY)
    if show:
        cv2.imshow("Binary Image", binary_image)
    return binary_image


def getContours(binary_image):
    # _, contours, hierarchy = cv2.findContours(binary_image,
    #                                         cv2.RETR_TREE,
    #                                         cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(binary_image.copy(),
                                           cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_contours(image, contours, image_name):
    index = -1
    thickness = 2
    color = (255, 0, 255)
    cv2.drawContours(image, contours, index, color, thickness)
    cv2.imshow(image_name, image)


def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros(
        [binary_image.shape[0], binary_image.shape[1], 3], 'uint8')

    for c in contours:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area > 20 and perimeter > 29):
            cv2.drawContours(rgb_image, [c], -1, (150, 250, 150), 1)
            cv2.drawContours(black_image, [c], -1, (150, 250, 150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
            cv2.circle(black_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
            print("Area: {}, Perimeter: {}".format(area, perimeter))
    print("number of contours {}".format(len(contours)))
    cv2.imshow("RGB Image Contours", rgb_image)
    cv2.imshow("Black Image Contours", black_image)


def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00'] != 0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx, cy


def detect_ball_in_a_frame(image_frame):
    # HSV encoding
    orangeLower = (0, 95, 100) # Lowest value to detect
    orangeUpper = (22, 255, 255) # Highest value to detect
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, orangeLower, orangeUpper)
    contours = getContours(binary_image_mask)
    draw_ball_contour(binary_image_mask, rgb_image, contours)


def main():
    # video_capture = cv2.VideoCapture('orange_ball.mp4')
    capture = VideoStream(src=0).start()
    time.sleep(2.0)
    fps = FPS().start()

    while True:
        frame = capture.read()
        frame = cv2.flip(frame, 1)
        detect_ball_in_a_frame(frame)
        time.sleep(0.033)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        fps.update()
    fps.stop()
    capture.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


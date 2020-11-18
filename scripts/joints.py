#!/usr/bin/env python3

import sys
import os
import cv2 as cv
import numpy as np


def main(args):
    file_path = args[1]
    file_name = os.path.split(file_path)[1]

    # Read image
    cv_image = cv.imread(file_path, 1)
    cv.imshow(file_name, cv_image)

    # Convert to HSV colour space
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

    # Get yellow mask
    y_mask = get_yellow_mask(hsv)
    y_moments = get_moments(y_mask, cv_image)
    y_centroid = get_centroid(y_moments)
    cv.waitKey(0)
    cv.destroyWindow('Yellow Joint')
    cv.destroyWindow('Contour')

    # Get blue mask
    b_mask = get_blue_mask(hsv)
    b_moments = get_moments(b_mask, cv_image)
    b_centroid = get_centroid(b_moments)
    cv.waitKey(0)
    cv.destroyWindow('Blue Joint')
    cv.destroyWindow('Contour')

    # Get green mask
    g_mask = get_green_mask(hsv)
    g_moments = get_moments(g_mask, cv_image)
    g_centroid = get_centroid(g_moments)
    cv.waitKey(0)
    cv.destroyWindow('Green Joint')
    cv.destroyWindow('Contour')

    # Get red mask
    r_mask = get_red_mask(hsv)
    r_moments = get_moments(r_mask, cv_image)
    r_centroid = get_centroid(r_moments)
    cv.waitKey(0)
    cv.destroyWindow('Red Joint')
    cv.destroyWindow('Contour')


def get_moments(mask, image=None):
    # Get contour
    contours, hierarchy = cv.findContours(mask, 1, 2)
    cnt = contours[0]

    # Get moments
    moments = cv.moments(cnt)

    # Show contour on image if provided
    if image is not None:
        cnt_image = cv.drawContours(image.copy(), [cnt], 0, (150, 255, 150), 1)
        cv.imshow('Contour', cnt_image)

    return moments


def get_centroid(moments):
    # Calculate centroid
    y_cx = moments['m10'] // moments['m00']
    y_cy = moments['m01'] // moments['m00']

    return np.array([y_cx, y_cy])


def get_yellow_mask(hsv):
    # Define blue range
    yellow_lower = np.array([25, 50, 50])
    yellow_upper = np.array([50, 255, 255])

    # Threshold image
    mask = cv.inRange(hsv, yellow_lower, yellow_upper)

    # Apply some smoothing
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.dilate(mask, kernel, iterations=1)

    # Show image section
    hsv_mask = cv.bitwise_and(hsv, hsv, mask=mask)
    bgr_mask = cv.cvtColor(hsv_mask, cv.COLOR_HSV2BGR)
    cv.imshow('Yellow Joint', bgr_mask)

    return mask


def get_blue_mask(hsv):
    # Define blue range
    blue_lower = np.array([110, 50, 50])
    blue_upper = np.array([150, 255, 255])

    # Threshold image
    mask = cv.inRange(hsv, blue_lower, blue_upper)

    # Apply some smoothing
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.dilate(mask, kernel, iterations=1)

    # Show image section
    hsv_mask = cv.bitwise_and(hsv, hsv, mask=mask)
    bgr_mask = cv.cvtColor(hsv_mask, cv.COLOR_HSV2BGR)
    cv.imshow('Blue Joint', bgr_mask)

    return mask

def get_green_mask(hsv):
    # Define blue range
    green_lower = np.array([50, 50, 50])
    green_upper = np.array([100, 255, 255])

    # Threshold image
    mask = cv.inRange(hsv, green_lower, green_upper)

    # Apply some smoothing
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.dilate(mask, kernel, iterations=1)

    # Show image section
    hsv_mask = cv.bitwise_and(hsv, hsv, mask=mask)
    bgr_mask = cv.cvtColor(hsv_mask, cv.COLOR_HSV2BGR)
    cv.imshow('Green Joint', bgr_mask)

    return mask


def get_red_mask(hsv):
    # Define blue range
    red_lower = np.array([0, 50, 50])
    red_upper = np.array([10, 255, 255])

    # Threshold image
    mask = cv.inRange(hsv, red_lower, red_upper)

    # Apply some smoothing
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.dilate(mask, kernel, iterations=1)

    # Show image section
    hsv_mask = cv.bitwise_and(hsv, hsv, mask=mask)
    bgr_mask = cv.cvtColor(hsv_mask, cv.COLOR_HSV2BGR)
    cv.imshow('Red Joint', bgr_mask)

    return mask


if __name__ == "__main__":
    main(sys.argv)

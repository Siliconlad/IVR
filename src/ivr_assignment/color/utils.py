import rospy
import cv2 as cv
import numpy as np


def shift(point, origin):
    new_point = np.zeros(point.shape, dtype=np.float64)
    new_point[0] = point[0] - origin[0]
    new_point[1] = origin[1] - point[1]

    return new_point


def centroid(m):
    # Calculate centroid
    y_cx = m['m10'] // m['m00']
    y_cy = m['m01'] // m['m00']

    return np.array([y_cx, y_cy])


def yellow_mask(hsv):
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


def blue_mask(hsv):
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


def green_mask(hsv):
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


def red_mask(hsv):
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

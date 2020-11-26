import cv2 as cv
import numpy as np


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

    return mask


def orange_mask(hsv):
    # Define orange range
    orange_lower = np.array([10, 50, 50])
    orange_upper = np.array([20, 255, 255])

    # Threshold image
    mask = cv.inRange(hsv, orange_lower, orange_upper)

    # Apply some smoothing
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.dilate(mask, kernel, iterations=1)

    return mask

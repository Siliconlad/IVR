import cv2 as cv
import numpy as np


def match(mask, template, image=None, c=0):
    res = cv.matchTemplate(mask, template, cv.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
    w, h = template.shape[::-1]
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)

    if image is not None:
        cv.rectangle(image, top_left, bottom_right, c, 2)

    # Calculate the quality of the matching
    score = np.sum(res[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]])
    # Calculate the center of the rectangle
    center = (np.array(top_left) + np.array(bottom_right)) / 2

    return center, score

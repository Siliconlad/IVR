import numpy as np


def shift(point, origin):
    new_point = np.zeros(point.shape, dtype=np.float64)
    new_point[0] = point[0] - origin[0]
    new_point[1] = origin[1] - point[1]

    return new_point


def centroid(m):
    # Calculate centroid
    cx = m['m10'] // m['m00']
    cy = m['m01'] // m['m00']

    return np.array([cx, cy])

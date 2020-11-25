import rospy
import numpy as np
import cv2 as cv

from scipy.cluster.vq import kmeans2, kmeans

import ivr_assignment.utils as ivr_utils
import ivr_assignment.utils.mask as ivr_mask
import ivr_assignment.utils.template as ivr_template


def get_red_joint(hsv, cv_image=None):
    r_mask = ivr_mask.red_mask(hsv)
    r_contour, _ = cv.findContours(r_mask, 1, 2)

    # No contour means the red joint is not visible
    if len(r_contour) > 0:
        r_center = np.zeros((2,), dtype=np.float64)

        # Average across all contour centers (in case joint is split)
        for contour in r_contour:
            moments = cv.moments(contour)
            r_center += ivr_utils.centroid(moments)

            # Add joint detection to image
            if cv_image is not None:
                cv_image = cv.drawContours(cv_image, [contour], 0, (150, 255, 150), 1)

        r_center = r_center / len(r_contour)
    else:
        r_center = None

    return r_center, cv_image


def get_green_joint(hsv, cv_image=None):
    g_mask = ivr_mask.green_mask(hsv)
    g_contour, _ = cv.findContours(g_mask, 1, 2)

    # No contour means the green joint is not visible
    if len(g_contour) > 0:
        g_center = np.zeros((2,), dtype=np.float64)

        # Average across all contour centers (in case joint is split)
        for contour in g_contour:
            moments = cv.moments(contour)
            g_center += ivr_utils.centroid(moments)

            # Add joint detection to image
            if cv_image is not None:
                cv_image = cv.drawContours(cv_image, [contour], 0, (150, 255, 150), 1)

        g_center = g_center / len(g_contour)
    else:
        g_center = None

    return g_center, cv_image


def get_blue_joint(hsv, cv_image=None):
    b_mask = ivr_mask.blue_mask(hsv)
    b_contour, _ = cv.findContours(b_mask, 1, 2)

    # No contour means the blue joint is not visible
    if len(b_contour) > 0:
        b_center = np.zeros((2,), dtype=np.float64)

        # Average across all contour centers (in case joint is split)
        for contour in b_contour:
            moments = cv.moments(contour)
            b_center += ivr_utils.centroid(moments)

            # Add joint detection to image
            if cv_image is not None:
                cv_image = cv.drawContours(cv_image, [contour], 0, (150, 255, 150), 1)

        b_center = b_center / len(b_contour)
    else:
        b_center = None

    return b_center, cv_image


def get_yellow_joint(hsv, cv_image=None):
    y_mask = ivr_mask.yellow_mask(hsv)
    y_contour, _ = cv.findContours(y_mask, 1, 2)

    # No contour means the yellow joint is not visible
    if len(y_contour) > 0:
        y_center = np.zeros((2,), dtype=np.float64)

        # Average across all contour centers (in case joint is split)
        for contour in y_contour:
            moments = cv.moments(contour)
            y_center += ivr_utils.centroid(moments)

            # Add joint detection to image
            if cv_image is not None:
                cv_image = cv.drawContours(cv_image, [contour], 0, (150, 255, 150), 1)

        y_center = y_center / len(y_contour)
    else:
        y_center = None

    return y_center, cv_image


def get_targets(hsv, template_sphere, template_box, cv_image=None):
    # TODO: Assumes at least one target is always visible

    ##########################
    #    Chamfer Matching    #
    ##########################

    # Get orange mask
    o_mask = ivr_mask.orange_mask(hsv)

    # Match templates
    sphere_center, sphere_score = ivr_template.match(o_mask, template_sphere, cv_image, 255)
    box_center, box_score = ivr_template.match(o_mask, template_box, cv_image, 0)

    # If templates have the same position, one of the objects is hidden
    if np.linalg.norm(sphere_center - box_center) < 5:
        if sphere_score < box_score:
            sphere_center = None
        else:
            box_center = None

    return sphere_center, box_center, cv_image

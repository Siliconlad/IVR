import rospy
import cv2 as cv
import ivr_assignment.joints.utils as joints_utils
import ivr_assignment.joints.utils.mask as joints_mask


def get_yellow(hsv):
    y_mask = joints_mask.yellow_mask(hsv)
    contours, hierarchy = cv.findContours(y_mask, 1, 2)

    if len(contours) == 0:
        rospy.logwarn("Yellow joint could not be detected in image 1!")
        return

    # Show contour on image if provided
    # cnt_image = cv.drawContours(self.image.copy(), [contours[0]], 0, (150, 255, 150), 1)
    # cv.imshow('Contour', cnt_image)

    mmts = cv.moments(contours[0])
    centroid = joints_utils.centroid(mmts)

    return centroid


def get_blue(hsv):
    b_mask = joints_mask.blue_mask(hsv)
    contours, hierarchy = cv.findContours(b_mask, 1, 2)

    if len(contours) == 0:
        rospy.logwarn("Blue joint could not be detected in image 1!")
        return

    # Show contour on image if provided
    # cnt_image = cv.drawContours(self.image.copy(), [contours[0]], 0, (150, 255, 150), 1)
    # cv.imshow('Contour', cnt_image)

    mmts = cv.moments(contours[0])
    centroid = joints_utils.centroid(mmts)

    return centroid


def get_green(hsv):
    g_mask = joints_mask.green_mask(hsv)
    contours, hierarchy = cv.findContours(g_mask, 1, 2)

    if len(contours) == 0:
        rospy.logwarn("Green joint could not be detected in image 1!")
        return

    # Show contour on image if provided
    # cnt_image = cv.drawContours(self.image.copy(), [contours[0]], 0, (150, 255, 150), 1)
    # cv.imshow('Contour', cnt_image)

    mmts = cv.moments(contours[0])
    centroid = joints_utils.centroid(mmts)

    return centroid


def get_red(hsv):
    r_mask = joints_mask.red_mask(hsv)
    contours, hierarchy = cv.findContours(r_mask, 1, 2)

    if len(contours) == 0:
        rospy.logwarn("Red joint could not be detected in image 1!")
        return

    # Show contour on image if provided
    # cnt_image = cv.drawContours(self.image.copy(), [contours[0]], 0, (150, 255, 150), 1)
    # cv.imshow('Contour', cnt_image)

    mmts = cv.moments(contours[0])
    centroid = joints_utils.centroid(mmts)

    return centroid

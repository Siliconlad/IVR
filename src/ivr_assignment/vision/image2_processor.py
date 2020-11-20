import math
import rospy
import cv2 as cv
import numpy as np

import ivr_assignment.utils as ivr_utils
import ivr_assignment.utils.mask as ivr_mask
import ivr_assignment.utils.template as ivr_template

from rospy import ROSInterruptException
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from ivr_assignment.msg import JointsStamped
from ivr_assignment.msg import PointStamped


class Image2Processor:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image2_processor')

        # Initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        #############################
        #    Process first image    #
        #############################

        msg = rospy.wait_for_message("/image2", Image)
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV color space
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        # Get green center
        g_mask = ivr_mask.green_mask(hsv)
        g_contour, _ = cv.findContours(g_mask, 1, 2)
        g_moments = cv.moments(g_contour[0])
        g_center = ivr_utils.centroid(g_moments)

        # Get blue center
        b_mask = ivr_mask.blue_mask(hsv)
        b_contour, _ = cv.findContours(b_mask, 1, 2)
        b_moments = cv.moments(b_contour[0])
        b_center = ivr_utils.centroid(b_moments)

        # Get yellow center
        y_mask = ivr_mask.yellow_mask(hsv)
        y_contour, _ = cv.findContours(y_mask, 1, 2)
        y_moments = cv.moments(y_contour[0])
        self.yellow = ivr_utils.centroid(y_moments)

        # Calculate the conversion from pixels to meters
        yb_ratio = 2.5 / np.sqrt(np.sum((self.yellow - b_center)**2))
        bg_ratio = 3.5 / np.sqrt(np.sum((b_center - g_center)**2))
        self.pixels_to_meters = (yb_ratio + bg_ratio) / 2

        ########################
        #    Configure Node    #
        ########################

        # Create publishers
        self.joints_pub = rospy.Publisher("/estimation/image2/joints", JointsStamped, queue_size=1)
        self.sphere_pub = rospy.Publisher("/estimation/image2/sphere", PointStamped, queue_size=1)

        # Create subscribers
        self.image_sub = rospy.Subscriber("/image2", Image, self.image_callback)

        # Load templates
        self.template_sphere = cv.imread('src/ivr_assignment/templates/template_sphere.png', 0)
        self.template_box = cv.imread('src/ivr_assignment/templates/template_box.png', 0)

    def image_callback(self, msg):
        # Get image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV color space
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        #########################
        #    Joint Detection    #
        #########################

        # TODO: Take all contours, find centroid, and take average (occurs for partially obscured joints)

        # Get red center
        r_mask = ivr_mask.red_mask(hsv)
        r_contour, _ = cv.findContours(r_mask, 1, 2)
        # No contour means the red joint is not visible
        if len(r_contour) > 0:
            r_moments = cv.moments(r_contour[0])
            r_center = ivr_utils.centroid(r_moments)
            # Add joint detection to image
            cv_image = cv.drawContours(cv_image, [r_contour[0]], 0, (150, 255, 150), 1)
        else:
            rospy.logwarn("Red joint could not be detected in image 2!")
            r_center = None

        # Get green center
        g_mask = ivr_mask.green_mask(hsv)
        g_contour, _ = cv.findContours(g_mask, 1, 2)
        # No contour means the green joint is not visible
        if len(g_contour) > 0:
            g_moments = cv.moments(g_contour[0])
            g_center = ivr_utils.centroid(g_moments)
            # Add joint detection to image
            cv_image = cv.drawContours(cv_image, [g_contour[0]], 0, (150, 255, 150), 1)
        else:
            rospy.logwarn("Green joint could not be detected in image 2!")
            g_center = None

        # Get blue center
        b_mask = ivr_mask.blue_mask(hsv)
        b_contour, _ = cv.findContours(b_mask, 1, 2)
        # No contour means the blue joint is not visible
        if len(b_contour) > 0:
            b_moments = cv.moments(b_contour[0])
            b_center = ivr_utils.centroid(b_moments)
            # Add joint detection to image
            cv_image = cv.drawContours(cv_image, [b_contour[0]], 0, (150, 255, 150), 1)
        else:
            rospy.logwarn("Blue joint could not be detected in image 2!")
            b_center = None

        # Publish positions
        self.publish_joints(r_center, g_center, b_center)

        ##########################
        #    Target Detection    #
        ##########################

        # Get orange mask
        o_mask = ivr_mask.orange_mask(hsv)

        # Match templates
        sphere_center, sphere_score = ivr_template.match(o_mask, self.template_sphere, cv_image, 255)
        box_center, box_score = ivr_template.match(o_mask, self.template_box, cv_image, 0)

        # If templates match one of the objects is hidden
        if np.linalg.norm(sphere_center - box_center) < 5:
            if sphere_score < box_score:
                rospy.logwarn("The SPHERE is hidden in image 2!")
                self.publish_sphere(None)
            else:
                self.publish_sphere(sphere_center)
        else:
            self.publish_sphere(sphere_center)

        ####################
        #    Show Image    #
        ####################

        cv.imshow('Processed Image', cv_image)
        cv.waitKey(1)

    def to_meters(self, centroid):
        if centroid is None:
            return

        # Shift centroids such that yellow is origin
        center = ivr_utils.shift(centroid, self.yellow)
        # Convert positions to meters
        center_m = self.pixels_to_meters * center

        return center_m

    def publish_joints(self, red, green, blue):
        # Create message
        pos = JointsStamped()
        pos.header.stamp = rospy.Time.now()

        # Set red joint position
        r_center_m = self.to_meters(red)
        pos.joints.red.x = r_center_m[0] if red is not None else math.nan
        pos.joints.red.y = math.nan
        pos.joints.red.z = r_center_m[1] if red is not None else math.nan
        pos.joints.red.hidden = (red is None)

        # Set green joint position
        g_center_m = self.to_meters(green)
        pos.joints.green.x = g_center_m[0] if green is not None else math.nan
        pos.joints.green.y = math.nan
        pos.joints.green.z = g_center_m[1] if green is not None else math.nan
        pos.joints.green.hidden = (green is None)

        # Set blue joint position
        b_center_m = self.to_meters(blue)
        pos.joints.blue.x = b_center_m[0] if blue is not None else math.nan
        pos.joints.blue.y = math.nan
        pos.joints.blue.z = b_center_m[1] if blue is not None else math.nan
        pos.joints.blue.hidden = (blue is None)

        # Publish results
        self.joints_pub.publish(pos)

    def publish_sphere(self, center):
        # Create message
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()

        # Set sphere position
        center_m = self.to_meters(center)
        msg.point.x = center_m[0] if center is not None else math.nan
        msg.point.y = math.nan
        msg.point.z = center_m[1] if center is not None else math.nan
        msg.point.hidden = (center is None)

        # Publish results
        self.sphere_pub.publish(msg)


def main():
    _ = Image2Processor()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()

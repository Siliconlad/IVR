import rospy
import cv2 as cv
import numpy as np

from rospy import ROSInterruptException
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from ivr_assignment.msg import JointsStamped
from ivr_assignment.color import utils


class Image1Joints:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image1_processor')

        # Create publishers
        self.pos_pub = rospy.Publisher("/estimation/image1/joints", JointsStamped, queue_size=1)

        # Initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        #######################
        # Process first image #
        #######################
        msg = rospy.wait_for_message("/image1", Image)
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV color space
        hsv = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        # Get centroids
        r_centroid = self.get_red(hsv)
        g_centroid = self.get_green(hsv)
        b_centroid = self.get_blue(hsv)
        self.yellow = self.get_yellow(hsv)

        # Calculate the conversion from pixels to meters
        yb_ratio = 2.5 / np.sqrt(np.sum((self.yellow - b_centroid)**2))
        bg_ratio = 3.5 / np.sqrt(np.sum((b_centroid - g_centroid)**2))
        self.pixels_to_meters = (yb_ratio + bg_ratio) / 2

        # Process centroids
        r_center_m = self.to_meters(r_centroid)
        g_center_m = self.to_meters(g_centroid)
        b_center_m = self.to_meters(b_centroid)

        # Publish positions
        self.publish_positions(r_center_m, g_center_m, b_center_m)
        ##################
        # End Processing #
        ##################

        # Create subscribers
        self.image_sub = rospy.Subscriber("/image1", Image, self.image_callback)

    def image_callback(self, msg):
        # Get image
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV color space
        hsv = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        # Get centroids
        r_centroid = self.get_red(hsv)
        g_centroid = self.get_green(hsv)
        b_centroid = self.get_blue(hsv)

        # Process centroids
        r_center_m = self.to_meters(r_centroid)
        g_center_m = self.to_meters(g_centroid)
        b_center_m = self.to_meters(b_centroid)

        # Publish positions
        self.publish_positions(r_center_m, g_center_m, b_center_m)

    def to_meters(self, centroid):
        # TODO: Deal with None values

        # Shift centroids such that yellow is origin
        center = utils.shift(centroid, self.yellow)

        # Convert positions to meters
        center_m = self.pixels_to_meters * center

        return center_m

    def get_yellow(self, hsv, show=False):
        mask = utils.yellow_mask(hsv)
        contours, hierarchy = cv.findContours(mask, 1, 2)

        # Show contour on image if provided
        if show:
            cnt_image = cv.drawContours(self.image.copy(), [contours[0]], 0, (150, 255, 150), 1)
            cv.imshow('Contour', cnt_image)

        # TODO: deal with case when there are no contours

        mmts = cv.moments(contours[0])
        centroid = utils.centroid(mmts)

        return centroid

    def get_blue(self, hsv, show=False):
        mask = utils.blue_mask(hsv)
        contours, hierarchy = cv.findContours(mask, 1, 2)

        # Show contour on image if provided
        if show:
            cnt_image = cv.drawContours(self.image.copy(), [contours[0]], 0, (150, 255, 150), 1)
            cv.imshow('Contour', cnt_image)

        # TODO: deal with case when there are no contours

        mmts = cv.moments(contours[0])
        centroid = utils.centroid(mmts)

        return centroid

    def get_green(self, hsv, show=False):
        mask = utils.green_mask(hsv)
        contours, hierarchy = cv.findContours(mask, 1, 2)

        # Show contour on image if provided
        if show:
            cnt_image = cv.drawContours(self.image.copy(), [contours[0]], 0, (150, 255, 150), 1)
            cv.imshow('Contour', cnt_image)

        # TODO: deal with case when there are no contours

        mmts = cv.moments(contours[0])
        centroid = utils.centroid(mmts)

        return centroid

    def get_red(self, hsv, show=False):
        mask = utils.red_mask(hsv)
        contours, hierarchy = cv.findContours(mask, 1, 2)

        # Show contour on image if provided
        if show:
            cnt_image = cv.drawContours(self.image.copy(), [contours[0]], 0, (150, 255, 150), 1)
            cv.imshow('Contour', cnt_image)

        # TODO: deal with case when there are no contours

        mmts = cv.moments(contours[0])
        centroid = utils.centroid(mmts)

        return centroid

    def publish_positions(self, red, green, blue, yellow=None):
        # Create message
        pos = JointsStamped()
        pos.header.stamp = rospy.Time.now()

        # Center of red joint in robot frame
        pos.joints.red.x = 0
        pos.joints.red.y = red[0]
        pos.joints.red.z = red[1]

        # Center of green joint in robot frame
        pos.joints.green.x = 0
        pos.joints.green.y = green[0]
        pos.joints.green.z = green[1]

        # Center of blue joint in robot frame
        pos.joints.blue.x = 0
        pos.joints.blue.y = blue[0]
        pos.joints.blue.z = blue[1]

        # Yellow joint is origin of robot frame
        pos.joints.yellow.x = 0
        pos.joints.yellow.y = 0 if yellow is None else yellow[0]
        pos.joints.yellow.z = 0 if yellow is None else yellow[1]

        # Publish results
        self.pos_pub.publish(pos)


def main():
    _ = Image1Joints()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()

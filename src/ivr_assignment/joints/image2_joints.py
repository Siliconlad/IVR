import rospy
import cv2 as cv
import numpy as np

import ivr_assignment.utils as ivr_utils
import ivr_assignment.utils.image as ivr_image

from rospy import ROSInterruptException
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from ivr_assignment.msg import JointsStamped


class Image2Joints:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image2_processor')

        # Create publishers
        self.pos_pub = rospy.Publisher("/estimation/image2/joints", JointsStamped, queue_size=1)

        # Initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        #######################
        # Process first image #
        #######################
        msg = rospy.wait_for_message("/image2", Image)
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV color space
        hsv = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        # Get centroids
        r_centroid = ivr_image.get_red(hsv)
        g_centroid = ivr_image.get_green(hsv)
        b_centroid = ivr_image.get_blue(hsv)
        self.yellow = ivr_image.get_yellow(hsv)

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
        self.image_sub = rospy.Subscriber("/image2", Image, self.image_callback)

    def image_callback(self, msg):
        # Get image
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV color space
        hsv = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        # Get centroids
        r_centroid = ivr_image.get_red(hsv)
        g_centroid = ivr_image.get_green(hsv)
        b_centroid = ivr_image.get_blue(hsv)

        # Process centroids
        r_center_m = self.to_meters(r_centroid)
        g_center_m = self.to_meters(g_centroid)
        b_center_m = self.to_meters(b_centroid)

        # Publish positions
        self.publish_positions(r_center_m, g_center_m, b_center_m)

    def to_meters(self, centroid):
        if centroid is None:
            return

        # Shift centroids such that yellow is origin
        center = ivr_utils.shift(centroid, self.yellow)

        # Convert positions to meters
        center_m = self.pixels_to_meters * center

        return center_m

    def publish_positions(self, red, green, blue, yellow=None):
        # Create message
        pos = JointsStamped()
        pos.header.stamp = rospy.Time.now()

        # Center of red joint in robot frame
        if red is not None:
            pos.joints.red.x = red[0]
            pos.joints.red.y = 0
            pos.joints.red.z = red[1]

        # Center of green joint in robot frame
        if green is not None:
            pos.joints.green.x = green[0]
            pos.joints.green.y = 0
            pos.joints.green.z = green[1]

        # Center of blue joint in robot frame
        if blue is not None:
            pos.joints.blue.x = blue[0]
            pos.joints.blue.y = 0
            pos.joints.blue.z = blue[1]

        # Yellow joint is origin of robot frame
        if yellow is not None:
            pos.joints.yellow.x = 0 if yellow is None else yellow[0]
            pos.joints.yellow.y = 0
            pos.joints.yellow.z = 0 if yellow is None else yellow[1]

        # Publish results
        self.pos_pub.publish(pos)


def main():
    _ = Image2Joints()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()

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
        self.yellow = self.get_yellow(hsv)
        b_centroid = self.get_blue(hsv)
        g_centroid = self.get_green(hsv)
        r_centroid = self.get_red(hsv)

        # Shift centroids such that yellow is origin
        g_center = utils.shift(g_centroid, self.yellow)
        b_center = utils.shift(b_centroid, self.yellow)
        r_center = utils.shift(r_centroid, self.yellow)

        # Calculate the conversion from pixels to meters
        yb_ratio = 2.5 / np.sqrt(np.sum(b_center**2))
        bg_ratio = 3.5 / np.sqrt(np.sum((b_center - g_center)**2))
        self.pixels_to_meters = (yb_ratio + bg_ratio) / 2

        # Convert positions to meters
        b_center_m = self.pixels_to_meters * b_center
        g_center_m = self.pixels_to_meters * g_center
        r_center_m = self.pixels_to_meters * r_center

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
        b_centroid = self.get_blue(hsv)
        g_centroid = self.get_green(hsv)
        r_centroid = self.get_red(hsv)

        # Shift centroids such that yellow is origin
        g_center = utils.shift(g_centroid, self.yellow)
        b_center = utils.shift(b_centroid, self.yellow)
        r_center = utils.shift(r_centroid, self.yellow)

        # Convert positions to meters
        b_center_m = self.pixels_to_meters * b_center
        g_center_m = self.pixels_to_meters * g_center
        r_center_m = self.pixels_to_meters * r_center

        # Publish positions
        self.publish_positions(r_center_m, g_center_m, b_center_m)

    def get_yellow(self, hsv):
        y_mask = utils.yellow_mask(hsv)
        y_moments = utils.moments(y_mask)
        y_centroid = utils.centroid(y_moments)

        return y_centroid

    def get_blue(self, hsv):
        b_mask = utils.blue_mask(hsv)
        b_moments = utils.moments(b_mask)
        b_centroid = utils.centroid(b_moments)

        return b_centroid

    def get_green(self, hsv):
        g_mask = utils.green_mask(hsv)
        g_moments = utils.moments(g_mask)
        g_centroid = utils.centroid(g_moments)

        return g_centroid

    def get_red(self, hsv):
        r_mask = utils.red_mask(hsv)
        r_moments = utils.moments(r_mask)
        r_centroid = utils.centroid(r_moments)

        return r_centroid

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

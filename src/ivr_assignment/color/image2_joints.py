import rospy
import cv2 as cv
import numpy as np

from rospy import ROSInterruptException
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from ivr_assignment.msg import JointsStamped
from ivr_assignment.color import utils


class Image2Joints:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image2_processor')

        # Class attributes
        self.image = None

        # Create publishers
        self.pos_pub = rospy.Publisher("/estimation/image2/joints", JointsStamped, queue_size=1)

        # Create subscribers
        self.image_sub = rospy.Subscriber("/image2", Image, self.image_callback)

        # Initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # Pixels to meters conversion ratio
        self.pixels_to_meters = None

    def image_callback(self, msg):
        # Get image
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert to HSV color space
        hsv = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        # Get yellow centroid
        y_mask = utils.yellow_mask(hsv)
        y_moments = utils.moments(y_mask)
        y_centroid = utils.centroid(y_moments)

        # Get blue centroid
        b_mask = utils.blue_mask(hsv)
        b_moments = utils.moments(b_mask)
        b_centroid = utils.centroid(b_moments)

        # Get green centroid
        g_mask = utils.green_mask(hsv)
        g_moments = utils.moments(g_mask)
        g_centroid = utils.centroid(g_moments)

        # Get red centroid
        r_mask = utils.red_mask(hsv)
        r_moments = utils.moments(r_mask)
        r_centroid = utils.centroid(r_moments)

        # Shift centroids such that yellow is origin
        g_center = utils.shift(g_centroid, y_centroid)
        b_center = utils.shift(b_centroid, y_centroid)
        r_center = utils.shift(r_centroid, y_centroid)

        # Calculate the conversion from pixels to meters
        if self.pixels_to_meters is None:
            # From yellow to blue
            yb_ratio = 2.5 / np.sqrt(np.sum(b_center ** 2))

            # From blue to green
            bg_ratio = 3.5 / np.sqrt(np.sum((b_center - g_center)**2))

            # Take the average for better results
            self.pixels_to_meters = (yb_ratio + bg_ratio) / 2

        # Convert positions to meters
        b_center_m = self.pixels_to_meters * b_center
        g_center_m = self.pixels_to_meters * g_center
        r_center_m = self.pixels_to_meters * r_center

        # Publish positions
        self.publish_positions(r_center_m, g_center_m, b_center_m)

    def publish_positions(self, red, green, blue, yellow=None):
        # Create message
        pos = JointsStamped()
        pos.header.stamp = rospy.Time.now()

        # Center of red joint in robot frame
        pos.joints.red.x = red[0]
        pos.joints.red.y = 0
        pos.joints.red.z = red[1]

        # Center of green joint in robot frame
        pos.joints.green.x = green[0]
        pos.joints.green.y = 0
        pos.joints.green.z = green[1]

        # Center of blue joint in robot frame
        pos.joints.blue.x = blue[0]
        pos.joints.blue.y = 0
        pos.joints.blue.z = blue[1]

        # Yellow joint is origin of robot frame
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
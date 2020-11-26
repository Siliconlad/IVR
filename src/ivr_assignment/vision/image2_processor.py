import rospy
import cv2 as cv
import numpy as np

import ivr_assignment.utils as ivr_utils
import ivr_assignment.utils.vision as ivr_vision

from rospy import ROSInterruptException
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from ivr_assignment.msg import StateStamped


# TODO: save position of blue joint in case it gets covered

class Image2Processor:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image2_processor')

        # Initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # Create publishers
        self.state_pub = rospy.Publisher("/image2/state", StateStamped, queue_size=1)

        # Create subscribers
        self.image_sub = rospy.Subscriber("/camera2/robot/image_raw", Image, self.image_callback)

        # Load templates
        self.template_sphere = cv.imread('src/ivr_assignment/templates/template_sphere.png', 0)
        self.template_box = cv.imread('src/ivr_assignment/templates/template_box_small.png', 0)

        # Save yellow joint position
        self.y_center = None

        # Save pixel to meter conversion ratio
        self.pixels_to_meters = None

    def image_callback(self, msg):
        # Get image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV color space
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        ##########################
        #    Object Detection    #
        ##########################

        # Get yellow center
        _, cv_image = ivr_vision.get_yellow_joint(hsv, cv_image)

        # Get red center
        r_center, cv_image = ivr_vision.get_red_joint(hsv, cv_image)

        # Get green center
        g_center, cv_image = ivr_vision.get_green_joint(hsv, cv_image)

        # Get blue center
        b_center, cv_image = ivr_vision.get_blue_joint(hsv, cv_image)

        # Get orange targets
        sphere_center, box_center, cv_image = ivr_vision.get_targets(hsv, self.template_sphere, self.template_box, cv_image)

        ####################################
        #    Set Yellow Joint to Center    #
        ####################################

        # Get yellow center
        if self.y_center is None:
            self.y_center, _ = ivr_vision.get_yellow_joint(hsv)

            # Calculate pixel to meter ratio
            yb_ratio = 2.5 / np.linalg.norm(self.y_center - b_center)
            bg_ratio = 3.5 / np.linalg.norm(b_center - g_center)
            self.pixels_to_meters = (yb_ratio + bg_ratio) / 2

        if r_center is not None:
            r_center = ivr_utils.shift(r_center, self.y_center)
            r_center *= self.pixels_to_meters
        else:
            rospy.logwarn("No RED joint in image 2!")

        if g_center is not None:
            g_center = ivr_utils.shift(g_center, self.y_center)
            g_center *= self.pixels_to_meters
        else:
            rospy.logwarn("No GREEN joint in image 2!")

        if b_center is not None:
            b_center = ivr_utils.shift(b_center, self.y_center)
            b_center *= self.pixels_to_meters
        else:
            rospy.logwarn("No BLUE joint in image 2!")

        if sphere_center is not None:
            sphere_center = ivr_utils.shift(sphere_center, self.y_center)
            sphere_center *= self.pixels_to_meters
        else:
            rospy.logwarn("No ORANGE SPHERE in image 2!")

        if box_center is not None:
            box_center = ivr_utils.shift(box_center, self.y_center)
            box_center *= self.pixels_to_meters
        else:
            rospy.logwarn("No ORANGE BOX in image 2!")

        # Publish positions
        self.publish_joints(r_center, g_center, b_center, sphere_center, box_center)

        ####################
        #    Show Image    #
        ####################

        cv.imshow('Processed Image 2', cv_image)
        cv.waitKey(1)

    def publish_joints(self, red, green, blue, sphere, box):
        # Create message
        msg = StateStamped()
        msg.header.stamp = rospy.Time.now()

        # Set red joint position
        msg.state.red.x = red[0] if red is not None else np.nan
        msg.state.red.y = np.nan
        msg.state.red.z = red[1] if red is not None else np.nan
        msg.state.red.hidden = (red is None)

        # Set green joint position
        msg.state.green.x = green[0] if green is not None else np.nan
        msg.state.green.y = np.nan
        msg.state.green.z = green[1] if green is not None else np.nan
        msg.state.green.hidden = (green is None)

        # Set blue joint position
        msg.state.blue.x = blue[0] if blue is not None else np.nan
        msg.state.blue.y = np.nan
        msg.state.blue.z = blue[1] if blue is not None else np.nan
        msg.state.blue.hidden = (blue is None)

        # Set sphere position
        msg.state.sphere.x = sphere[0] if sphere is not None else np.nan
        msg.state.sphere.y = np.nan
        msg.state.sphere.z = sphere[1] if sphere is not None else np.nan
        msg.state.sphere.hidden = (sphere is None)

        # Set box position
        msg.state.box.x = box[0] if box is not None else np.nan
        msg.state.box.y = np.nan
        msg.state.box.z = box[1] if box is not None else np.nan
        msg.state.box.hidden = (box is None)

        # Publish results
        self.state_pub.publish(msg)


def main():
    _ = Image2Processor()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")

    cv.destroyAllWindows()

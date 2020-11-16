#!/usr/bin/env python3

import rospy
import cv2 as cv

from rospy import ROSInterruptException
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from ivr_assignment.msg import TargetsStamped


class Image2Target:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image2_target')

        # Class attributes
        self.image = None

        # Create publishers
        self.pos_pub = rospy.Publisher("/estimation/image2/targets", TargetsStamped, queue_size=1)

        # Create subscribers
        self.image_sub = rospy.Subscriber("/image_topic2", Image, self.image_callback)

        # Initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Get image
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # TODO: Image processing

        # Publish results
        pos = TargetsStamped()
        pos.header.stamp = rospy.Time.now()
        pos.sphere = None
        pos.cuboid = None
        self.pos_pub.publish(pos)


def main():
    _ = Image2Target()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")


if __name__ == '__main__':
    main()
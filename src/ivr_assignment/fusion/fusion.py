#!/usr/bin/env python3

import rospy
import message_filters
import cv2 as cv

from rospy import ROSInterruptException
from cv_bridge import CvBridge, CvBridgeError

from ivr_assignment.msg import JointsStamped
from geometry_msgs.msg import PointStamped


class Fusion:

    def __init__(self):
        # Initialize the node
        rospy.init_node('fusion')

        # Class attributes

        # Create publishers
        self.joints_pub = rospy.Publisher("/estimation/joints", JointsStamped, queue_size=1)
        self.sphere_pub = rospy.Publisher("/estimation/sphere", PointStamped, queue_size=1)

        # Set up message filtering
        self.image1_joints_sub = message_filters.Subscriber('/estimation/image1/joints', JointsStamped)
        self.image2_joints_sub = message_filters.Subscriber('/estimation/image2/joints', JointsStamped)

        self.image1_targets_sub = message_filters.Subscriber('/estimation/image1/sphere', PointStamped)
        self.image2_targets_sub = message_filters.Subscriber('/estimation/image2/sphere', PointStamped)

        ts = message_filters.TimeSynchronizer([self.image1_joints_sub, self.image2_joints_sub,
                                               self.image1_targets_sub, self.image2_targets_sub])
        ts.registerCallback(self.callback)

    def callback(self, image1_joints, image2_joints, image1_targets, image2_targets):
        # TODO: Process points

        # Publish final estimated joint positions
        joints = JointsStamped()
        joints.header.stamp = rospy.Time.now()
        self.joints_pub.publish(joints)

        # Publish final estimated sphere position
        sphere = PointStamped()
        sphere.header.stamp = rospy.Time.now()
        self.sphere_pub.publish(sphere)


def main():
    _ = Fusion()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rospy
import message_filters
import cv2 as cv

from rospy import ROSInterruptException
from cv_bridge import CvBridge, CvBridgeError

from ivr_assignment.msg import JointsStamped, TargetsStamped, StateStamped


class Fusion:

    def __init__(self):
        # Initialize the node
        rospy.init_node('fusion')

        # Class attributes

        # Create publishers
        self.state_pub = rospy.Publisher("/estimation/pos", StateStamped, queue_size=1)

        # Set up message filtering
        self.image1_joints_sub = message_filters.Subscriber('/estimation/image1/joints', JointsStamped)
        self.image2_joints_sub = message_filters.Subscriber('/estimation/image2/joints', JointsStamped)

        self.image1_targets_sub = message_filters.Subscriber('/estimation/image1/targets', TargetsStamped)
        self.image2_targets_sub = message_filters.Subscriber('/estimation/image2/targets', TargetsStamped)

        ts = message_filters.TimeSynchronizer([self.image1_joints_sub, self.image2_joints_sub,
                                               self.image1_targets_sub, self.image2_targets_sub])
        ts.registerCallback(self.callback)

    def callback(self, image1_joints, image2_joints, image1_targets, image2_targets):
        # TODO: Process points

        # Publish result
        state = StateStamped()
        state.header.stamp = rospy.Time.now()

        self.state_pub.publish(state)


def main():
    _ = Fusion()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")


if __name__ == '__main__':
    main()
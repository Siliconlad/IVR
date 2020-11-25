import rospy
import numpy as np

from rospy import ROSInterruptException
from ivr_assignment.msg import StateStamped, AnglesStamped


class JointAngles:

    def __init__(self):
        # Initialize the node
        rospy.init_node('joint_angles')

        # Create publishers
        self.angles_pub = rospy.Publisher("/fusion/angles", AnglesStamped, queue_size=1)

        # Create subscribers
        self.joints_sub = rospy.Subscriber("/fusion/state", StateStamped, self.joints_callback)
        #self.joints_sub = rospy.Subscriber("/fk/state", StateStamped, self.joints_callback)

        # Store previous angles
        self.theta_2 = 0
        self.theta_3 = 0
        self.theta_4 = 0
        self.theta_4_sign = 1

        self.true_angle = 0

    def joints_callback(self, msg):
        # Get position of red joint
        r_x = msg.state.red.x
        r_y = msg.state.red.y
        r_z = msg.state.red.z

        # Get position of green joint
        g_x = msg.state.green.x
        g_y = msg.state.green.y
        g_z = msg.state.green.z

        # Get theta_2
        if (g_z - 2.5) != 0:
            theta_2 = np.arctan2(-g_y, g_z - 2.5)
        else:
            theta_2 = self.theta_2

        # Get theta_3
        if (g_z - 2.5) != 0:
            theta_3 = np.arctan2(g_x * np.cos(theta_2), g_z - 2.5)
        else:
            theta_3 = self.theta_3

        # Store results
        self.theta_2 = theta_2
        self.theta_3 = theta_3

        # Publish results
        self.publish_angles(0, theta_2, theta_3, np.nan)

    def true_angle_callback(self, msg):
        self.true_angle = msg.data

    def publish_angles(self, theta_1, theta_2, theta_3, theta_4):
        msg = AnglesStamped()
        msg.header.stamp = rospy.Time.now()

        msg.angles.theta_1 = theta_1
        msg.angles.theta_2 = theta_2
        msg.angles.theta_3 = theta_3
        msg.angles.theta_4 = theta_4

        self.angles_pub.publish(msg)

    def norm(self, value):
        if value < -1:
            rospy.logwarn("NORMED: {}".format(value))
            value = -1
        elif value > 1:
            rospy.logwarn("NORMED: {}".format(value))
            value = 1

        return value


def main():
    _ = JointAngles()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")

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

        # Store previous angles
        self.theta_2 = 0
        self.theta_3 = 0
        self.theta_4 = 0
        self.theta_4_sign = 1

    def joints_callback(self, msg):
        r = msg.state.red
        g = msg.state.green

        # Get second angle of blue joint (rotation about x)
        try:
            theta_3 = np.arcsin(self.norm(g.x / 3.5))
        except ZeroDivisionError:
            theta_3 = self.theta_3

        # Get first angle of blue joint (rotation about y)
        try:
            theta_2 = np.arcsin(self.norm(-1 * g.y / (3.5 * np.cos(theta_3))))
        except ZeroDivisionError:
            theta_2 = self.theta_2

        # Get angle of green joint
        try:
            theta_4 = np.arccos(self.norm((3.5 * (r.x - g.x)) / (3 * g.x)))
            sin_theta_4 = ((r.y - g.y) + 3 * np.sin(theta_2) * np.cos(theta_3) * np.cos(theta_4)) / (3 * np.cos(theta_3))

            if sin_theta_4 != np.nan:
                theta_4_sign = np.arcsin(self.norm(sin_theta_4))
                # Prevents the sign from randomly switching when close to zero
                if not np.allclose(theta_4_sign, 0):
                    self.theta_4_sign = -1 * theta_4_sign

            # Required because the arccos doesn't give sign of the angle
            theta_4 *= np.sign(self.theta_4_sign)
        except ZeroDivisionError:
            theta_4 = self.theta_4

        # Store angles
        self.theta_2 = theta_2
        self.theta_3 = theta_3
        self.theta_4 = theta_4

        # Publish results
        self.publish_angles(0, theta_2, theta_3, theta_4)

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

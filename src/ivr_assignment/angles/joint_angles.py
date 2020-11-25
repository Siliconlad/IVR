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
        #self.joints_sub = rospy.Subscriber("/fk/joints", JointsStamped, self.joints_callback)

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

        # Get second angle of blue joint (rotation about x)
        try:
            theta_3 = np.arcsin(self.norm(g_x / 3.5))
        except ZeroDivisionError:
            theta_3 = self.theta_3

        # Get first angle of blue joint (rotation about y)
        try:
            theta_2 = np.arcsin(self.norm(-1 * g_y / (3.5 * np.cos(theta_3))))
        except ZeroDivisionError:
            theta_2 = self.theta_2

        # Get angle of green joint
        try:
            theta_4 = np.arccos(self.norm((3.5 * (r_x - g_x)) / (3 * g_x)))
            sin_theta_4 = ((r_y - g_y) + 3 * np.sin(theta_2) * np.cos(theta_3) * np.cos(theta_4) ) / (3 * np.cos(theta_3))

            if sin_theta_4 != np.nan:
                theta_4_sign = np.arcsin(self.norm(sin_theta_4))
                # Prevents the sign from randomly switching when close to zero
                if not np.allclose(theta_4_sign, 0):
                    self.theta_4_sign = -1 * theta_4_sign

            theta_4 *= np.sign(self.theta_4_sign)
        except ZeroDivisionError:
            theta_4 = self.theta_4

        # Store angles
        self.theta_2 = theta_2
        self.theta_3 = theta_3
        self.theta_4 = theta_4

        # Publish results
        self.publish_angles(0, theta_2, theta_3, theta_4)

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

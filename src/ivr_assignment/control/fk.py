import rospy
import message_filters
import numpy as np
from numpy import sin, cos

from rospy import ROSInterruptException
from std_msgs.msg import Float64
from ivr_assignment.msg import FloatStamped, JointsStamped

class ForwardKinematics:

    def __init__(self):
        # Initialize the node
        rospy.init_node('forward_kinematics')

        # Create publishers
        self.green_pub = rospy.Publisher("/fk/joints", JointsStamped, queue_size=1)

        # Create subscriber
        self.joint2_sub = message_filters.Subscriber('/joint2/angle', FloatStamped)
        self.joint3_sub = message_filters.Subscriber('/joint3/angle', FloatStamped)
        self.joint4_sub = message_filters.Subscriber('/joint4/angle', FloatStamped)

        ts = message_filters.ApproximateTimeSynchronizer([self.joint2_sub, self.joint3_sub, self.joint4_sub],
                                                         queue_size=1, slop=0.05)
        ts.registerCallback(self.callback)

        # Set rate to 10Hz
        self.rate = rospy.Rate(100)

    def callback(self, joint2, joint3, joint4):
        theta_1 = 0
        theta_2 = joint2.angle
        theta_3 = joint3.angle
        theta_4 = joint4.angle

        a_0_1 = np.array([[ sin(theta_1),  0, cos(theta_1),   0],
                          [-cos(theta_1),  0, sin(theta_1),   0],
                          [            0, -1,            0, 2.5],
                          [            0,  0,            0,   1]])

        a_1_2 = np.array([[ sin(theta_2), 0, -cos(theta_2), 0],
                          [-cos(theta_2), 0, -sin(theta_2), 0],
                          [            0, 1,             0, 0],
                          [            0,  0,             0, 1]])

        a_2_3 = np.array([[cos(theta_3),  0, -sin(theta_3), 3.5*cos(theta_3)],
                          [sin(theta_3),  0,  cos(theta_3), 3.5*sin(theta_3)],
                          [           0, -1,             0,                0],
                          [           0,  0,             0,                1]])

        a_3_4 = np.array([[ cos(theta_4), -sin(theta_4), 0, 3*cos(theta_4)],
                          [ sin(theta_4),  cos(theta_4), 0, 3*sin(theta_4)],
                          [            0,             0, 1,              0],
                          [            0,             0, 0,              1]])

        ## Multiply
        a_0_2 = np.matmul(a_0_1, a_1_2)
        a_0_3 = np.matmul(a_0_2, a_2_3)
        a_0_4 = np.matmul(a_0_3, a_3_4)

        # Publish position
        msg = JointsStamped()
        msg.header.stamp = rospy.Time.now()

        # Red position
        r_x, r_y, r_z, _ = a_0_4[:, -1].reshape(-1)
        msg.joints.red.x = r_x
        msg.joints.red.y = r_y
        msg.joints.red.z = r_z

        # Green position
        g_x, g_y, g_z, _ = a_0_3[:, -1].reshape(-1)
        msg.joints.green.x = g_x
        msg.joints.green.y = g_y
        msg.joints.green.z = g_z

        self.green_pub.publish(msg)

def main():
    _ = ForwardKinematics()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")


if __name__ == '__main__':
    main()

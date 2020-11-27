import rospy
from numpy import sin, pi

from rospy import ROSInterruptException
from std_msgs.msg import Float64
from ivr_assignment.msg import AnglesStamped


class JointController:

    def __init__(self):
        # Initialize the node
        rospy.init_node('joint_controller')

        # Create publishers
        self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=1)
        self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=1)
        self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=1)

        self.joints_pub = rospy.Publisher("/joints", AnglesStamped, queue_size=1)

        # Set rate to 100Hz
        self.rate = rospy.Rate(100)

        # Get start time
<<<<<<< HEAD
        self.start_time = None;
=======
        self.start_time = None
>>>>>>> 4122c9adfba79c4cedf7d17d4b07545e5ec03a4a

    def loop(self):
        while not rospy.is_shutdown():
            if self.start_time is None:
                self.start_time = rospy.get_time()

            current_time = rospy.get_time() - self.start_time

            # Calculate the angles
            theta2 = (pi/2) * sin((pi/15) * current_time)
            theta3 = (pi/2) * sin((pi/18) * current_time)
            theta4 = (pi/3) * sin((pi/20) * current_time)

            # Publish joints to robot
            theta2_msg = Float64()
            theta2_msg.data = theta2
            self.joint2_pub.publish(theta2_msg)

            theta3_msg = Float64()
            theta3_msg.data = theta3
            self.joint3_pub.publish(theta3_msg)

            theta4_msg = Float64()
            theta4_msg.data = theta4
            self.joint4_pub.publish(theta4_msg)

            # Publish joints to topic
            angles_msg = AnglesStamped()
            angles_msg.header.stamp = rospy.Time.now()
            angles_msg.angles.theta_1 = 0
            angles_msg.angles.theta_2 = theta2
            angles_msg.angles.theta_3 = theta3
            angles_msg.angles.theta_4 = theta4
            self.joints_pub.publish(angles_msg)

            # Sleep
            self.rate.sleep()


def main():
    j_controller = JointController()

    try:
        j_controller.loop()
    except ROSInterruptException:
        print("Shutting down...")


if __name__ == '__main__':
    main()

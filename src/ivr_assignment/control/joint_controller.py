import math
import rospy

from rospy import ROSInterruptException
from std_msgs.msg import Float64
from ivr_assignment.msg import FloatStamped


class JointController:

    def __init__(self):
        # Initialize the node
        rospy.init_node('joint_controller')

        # Create publishers
        self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=1)
        self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=1)
        self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=1)

        self.joint2_truth_pub = rospy.Publisher("/joint2/angle", FloatStamped, queue_size=1)
        self.joint3_truth_pub = rospy.Publisher("/joint3/angle", FloatStamped, queue_size=1)
        self.joint4_truth_pub = rospy.Publisher("/joint4/angle", FloatStamped, queue_size=1)

        # Set rate to 10Hz
        self.rate = rospy.Rate(100)

        # Get start time
        self.start_time = rospy.get_time()

    def loop(self):
        while not rospy.is_shutdown():
            current_time = rospy.get_time() - self.start_time

            # Publish angle for joint 2
            joint2_angle = Float64()
            joint2_angle.data = (math.pi/2) * math.sin((math.pi/15) * current_time)
            self.joint2_pub.publish(joint2_angle)

            joint2_angle = FloatStamped()
            joint2_angle.angle = (math.pi/2) * math.sin((math.pi/15) * current_time)
            joint2_angle.header.stamp = rospy.Time.now()
            self.joint2_truth_pub.publish(joint2_angle)

            # Publish angle for joint 3
            joint3_angle = Float64()
            joint3_angle.data = (math.pi/2) * math.sin((math.pi/18) * current_time)
            self.joint3_pub.publish(joint3_angle)
            self.joint3_pub.publish(joint3_angle)

            joint3_angle = FloatStamped()
            joint3_angle.angle = (math.pi/2) * math.sin((math.pi/18) * current_time)
            joint3_angle.header.stamp = rospy.Time.now()
            self.joint3_truth_pub.publish(joint3_angle)

            # Publish angle for joint 4
            joint4_angle = Float64()
            joint4_angle.data = (math.pi/3) * math.sin((math.pi/20) * current_time)
            self.joint4_pub.publish(joint4_angle)

            joint4_angle = FloatStamped()
            joint4_angle.angle = (math.pi/3) * math.sin((math.pi/20) * current_time)
            joint4_angle.header.stamp = rospy.Time.now()
            self.joint4_truth_pub.publish(joint4_angle)

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

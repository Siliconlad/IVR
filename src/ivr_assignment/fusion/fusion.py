import rospy
import message_filters

from rospy import ROSInterruptException
from ivr_assignment.msg import JointsStamped


class Fusion:

    def __init__(self):
        # Initialize the node
        rospy.init_node('fusion')

        # Create publishers
        self.joints_pub = rospy.Publisher("/estimation/joints", JointsStamped, queue_size=1)

        # Set up message filtering
        self.image1_joints_sub = message_filters.Subscriber('/estimation/image1/joints', JointsStamped)
        self.image2_joints_sub = message_filters.Subscriber('/estimation/image2/joints', JointsStamped)

        ts = message_filters.ApproximateTimeSynchronizer([self.image1_joints_sub, self.image2_joints_sub], queue_size=1, slop=0.05)
        ts.registerCallback(self.callback)

    def callback(self, image1_joints, image2_joints):
        # Publish final estimated joint positions
        joints = JointsStamped()
        joints.header.stamp = rospy.Time.now()

        # Yellow joint is the origin
        joints.joints.yellow.x = 0
        joints.joints.yellow.y = 0
        joints.joints.yellow.z = 0

        # Blue joint position
        joints.joints.blue.x = image2_joints.joints.blue.x
        joints.joints.blue.y = image1_joints.joints.blue.y
        joints.joints.blue.z = (image1_joints.joints.blue.z + image2_joints.joints.blue.z) / 2

        # Green joint position
        joints.joints.green.x = image2_joints.joints.green.x
        joints.joints.green.y = image1_joints.joints.green.y
        joints.joints.green.z = (image1_joints.joints.green.z + image2_joints.joints.green.z) / 2

        # Red joint position
        joints.joints.red.x = image2_joints.joints.red.x
        joints.joints.red.y = image1_joints.joints.red.y
        joints.joints.red.z = (image1_joints.joints.red.z + image2_joints.joints.red.z) / 2

        self.joints_pub.publish(joints)


def main():
    _ = Fusion()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")


if __name__ == '__main__':
    main()
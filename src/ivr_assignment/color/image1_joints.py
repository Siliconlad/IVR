import rospy
import cv2 as cv

from rospy import ROSInterruptException
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from ivr_assignment.msg import JointsStamped


class Image2Joints:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image1_processor')

        # Class attributes
        self.image = None

        # Create publishers
        self.pos_pub = rospy.Publisher("/estimation/image1/joints", JointsStamped, queue_size=1)

        # Create subscribers
        self.image_sub = rospy.Subscriber("/image_topic1", Image, self.image_callback)

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
        pos = JointsStamped()
        pos.header.stamp = rospy.Time.now()
        pos.yellow = None
        pos.blue = None
        pos.green = None
        pos.red = None
        self.pos_pub.publish(pos)


def main():
    _ = Image2Joints()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")


if __name__ == '__main__':
    main()
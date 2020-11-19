import rospy
import cv2 as cv

from rospy import ROSInterruptException
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped


class Image1Target:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image1_target')

        # Class attributes
        self.image = None

        # Create publishers
        self.sphere_pub = rospy.Publisher("/estimation/image1/sphere", PointStamped, queue_size=1)

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

        # Publish estimated position of sphere
        pos = PointStamped()
        pos.header.stamp = rospy.Time.now()
        pos.point = None
        self.sphere_pub.publish(pos)


def main():
    _ = Image1Target()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")


if __name__ == '__main__':
    main()
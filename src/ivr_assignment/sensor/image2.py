import rospy
import cv2 as cv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image2_processing')

        # Class attributes
        self.cv_image2 = None

        # Create publisher
        self.image2_pub = rospy.Publisher("image2", Image, queue_size=1)

        # Create subscriber
        self.image2_sub = rospy.Subscriber("/camera2/robot/image_raw", Image, self.image2_callback)

        # Initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    # Receive data, process it, and publish
    def image2_callback(self, data):
        # Receive the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Uncomment if you want to save the image
        # cv.imwrite('image_copy.png', cv_image2)

        _ = cv.imshow('Image 2', self.cv_image2)
        cv.waitKey(1)

        # Publish the results
        try:
            self.image2_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main():
    _ = ImageConverter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()



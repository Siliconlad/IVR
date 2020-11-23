import rospy
import cv2 as cv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:

    def __init__(self):
        # Initialize the node
        rospy.init_node('image1_processing')

        # Class attributes
        self.cv_image1 = None

        # Create publisher
        self.image1_pub = rospy.Publisher("image1", Image, queue_size=1)

        # Create subscriber
        self.image1_sub = rospy.Subscriber("/camera1/robot/image_raw", Image, self.image1_callback)

        # Initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    # Receive data from camera 1, process it, and publish
    def image1_callback(self, data):
        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
        # Uncomment if you want to save the image
        # cv.imwrite('image_copy.png', cv_image1)

        # Uncomment if you want to show the image
        #_ = cv.imshow('Image 1', self.cv_image1)
        #cv.waitKey(1)

        # Publish the results
        try:
            self.image1_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
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



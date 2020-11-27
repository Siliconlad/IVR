import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

from ivr_assignment.msg import StateStamped, State
from ivr_assignment.utils import jacobian, fk

class Controller:

    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.bridge = CvBridge()

        self.sphere_sub = rospy.Subscriber("/fusion/state", StateStamped, self.callback)
        self.angles = [0.0, 0.0, 0.0, 0.0]

        self.time_trajectory = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')     
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')   
        # initialize error and derivative of error for trajectory tracking  
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_i = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')

        self.rate = rospy.Rate(10)

    def control_closed(self, angles, desired_position):
        # P gain
        K_p = np.eye(3) * 0.01
        # I gain
        K_i = np.eye(3) * 0
        # D gain
        K_d = np.eye(3) * 0
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step + 1
        self.time_previous_step = cur_time
        # robot end-effector position
        pos, _ = fk(angles) # self.detect_end_effector(image)
        #print('\npos:', pos)
        # desired trajectory
        pos_d = desired_position # self.trajectory()
        #print('desired pos:', desired_position)
        #print(pos_d)
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error) / dt
        # estimate error
        self.error = pos_d - pos
        # estimate integral of error
        self.error_i = self.error * dt
        q = np.array(angles).T # self.detect_joint_angles(image) # estimate initial value of joints'
        J_inv = np.linalg.pinv(jacobian(angles)) # np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_i, self.error_i.transpose()) + np.dot(K_p, self.error.transpose())))  # control input (angular velocity of joints)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        #print('q:', q_d)
        return q_d

    def callback(self, data):
        sphere = data.state.sphere
        sphere_pos = [sphere.x, sphere.y, sphere.z]

        q_d = self.control_closed(self.angles, sphere_pos)
        self.angles = q_d
        #q_d = self.control_open(cv_image)
        self.joint1=Float64()
        self.joint1.data= q_d[0]
        self.joint2=Float64()
        self.joint2.data= q_d[1]
        self.joint3=Float64()
        self.joint3.data= q_d[2]
        self.joint4=Float64()
        self.joint4.data= q_d[3]

        self.robot_joint1_pub.publish(self.joint1)
        self.robot_joint2_pub.publish(self.joint2)
        self.robot_joint3_pub.publish(self.joint3)
        self.robot_joint4_pub.publish(self.joint4)


def main():
  c = Controller()
  try:
    rospy.spin()
    # c.loop()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

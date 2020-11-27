import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64

from ivr_assignment.msg import StateStamped, PointStamped
from ivr_assignment.utils import jacobian, fk


class Controller:

    def __init__(self):
        rospy.init_node('controller')

        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=1)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=1)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=1)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=1)

        self.sphere_sub = rospy.Subscriber("/fusion/state", StateStamped, self.callback)
        self.angles = np.array([0.0, 0.0, 0.0, 0.0])

        # Time
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')

        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([[0.0, 0.0, 0.0]], dtype='float64')
        self.error_d = np.array([[0.0, 0.0, 0.0]], dtype='float64')

    def control_closed(self, angles, desired_position, current_position):
        # P gain
        K_p = np.eye(3) * 1.9
        # D gain
        K_d = np.eye(3) * 0

        # Calculate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time

        # Robot end-effector position
        # pos, _ = fk(angles)
        # pos = pos.reshape(1, -1)
        pos = np.array(current_position).reshape(1, -1)

        # Desired trajectory
        pos_d = np.array(desired_position).reshape(1, -1)

        # Estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error) / dt
        # Estimate error
        self.error = pos_d - pos

        J_inv = np.linalg.pinv(jacobian(angles))
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.T) + np.dot(K_p, self.error.T)))

        q = angles.reshape(-1, 1)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)

        return q_d

    def callback(self, data):
        sphere = data.state.sphere
        red = data.state.red
        red_pos = [red.x, red.y, red.z]
        sphere_pos = [sphere.x, sphere.y, sphere.z]

        q_d = self.control_closed(self.angles, sphere_pos, red_pos)
        self.angles = q_d.reshape(-1,)
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
  _ = Controller()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

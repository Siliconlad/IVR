import rospy
import message_filters
import numpy as np

from rospy import ROSInterruptException
from ivr_assignment.msg import StateStamped


class Fusion:

    def __init__(self):
        # Initialize the node
        rospy.init_node('fusion')

        # Initial position of the objects
        self.red = np.array([0, 0, 9])
        self.green = np.array([0, 0, 6])
        self.blue = np.array([0, 0, 2.5])
        self.sphere = np.array([0, 0, 0])  # TODO: Does this cause a problem?

        # Create publishers
        self.state_pub = rospy.Publisher("/fusion/state", StateStamped, queue_size=1)

        # Set up message filtering
        self.image1_state_sub = message_filters.Subscriber('/image1/state', StateStamped)
        self.image2_state_sub = message_filters.Subscriber('/image2/state', StateStamped)

        ts = message_filters.ApproximateTimeSynchronizer([self.image1_state_sub, self.image2_state_sub],
                                                         queue_size=1, slop=0.017)
        ts.registerCallback(self.callback)

    def callback(self, image1_state, image2_state):
        # Extract red joint
        red1 = image1_state.state.red
        red2 = image2_state.state.red
        # Extract green joint
        green1 = image1_state.state.green
        green2 = image2_state.state.green
        # Extract blue joint
        blue1 = image1_state.state.blue
        blue2 = image2_state.state.blue
        # Extract target sphere
        sphere1 = image1_state.state.sphere
        sphere2 = image2_state.state.sphere
        # Extract box
        box1 = image1_state.state.box
        box2 = image2_state.state.box

        ###################
        #    Red Joint    #
        ###################

        # If we have perfect visibility of the joint
        if not red1.hidden and not red2.hidden:
            r_x = (0.7 * red2.x) + (0.3 * self.red[0])
            r_y = (0.7 * red1.y) + (0.3 * self.red[1])
            r_z = (0.5 * red1.z) + (0.5 * red2.z)

            r_center = np.array([r_x, r_y, r_z])

        # If the red joint is hidden in image1 but not in image2
        elif red1.hidden and not red2.hidden:
            objects = []
            diff = []

            # Calculate distances
            for obj in [green1, blue1, sphere1, box1]:
                if not obj.hidden:
                    objects.append(obj)

                    dy = self.red[1] - obj.y
                    dz = red2.z - obj.z
                    diff.append(np.linalg.norm([dy, dz]))

            if len(objects) == 0:
                rospy.logerr("Image 1 detected nothing!")
                r_y = self.red[1]
            else:
                diff_min = np.argmin(diff)
                r_y = objects[diff_min].y

            # Calculate center of red joint
            r_x = red2.x
            r_y = (0.5 * r_y) + (0.5 * self.red[1])
            r_z = red2.z
            r_center = np.array([r_x, r_y, r_z])

        # If the red joint is hidden in image2 but not in image1
        elif not red1.hidden and red2.hidden:
            objects = []
            diff = []

            # Calculate distances
            for obj in [green2, blue2, sphere2, box2]:
                if not obj.hidden:
                    objects.append(obj)

                    dx = self.red[0] - obj.x
                    dz = red1.z - obj.z
                    diff.append(np.linalg.norm([dx, dz]))

            if len(objects) == 0:
                rospy.logerr("Image 2 detected nothing!")
                r_x = self.red[0]
            else:
                diff_min = np.argmin(diff)
                r_x = objects[diff_min].x

            # Calculate center of red joint
            r_x = (0.5 * r_x) + (0.5 * self.red[0])
            r_y = red1.y
            r_z = red1.z
            r_center = np.array([r_x, r_y, r_z])

        # If the red joint is completely hidden
        else:
            rospy.logwarn("Red joint is completely hidden! Using previous position...")
            r_center = self.red

        #####################
        #    Green Joint    #
        #####################

        # If we have perfect visibility of the joint
        if not green1.hidden and not green2.hidden:
            g_x = (0.7 * green2.x) + (0.3 * self.green[0])
            g_y = (0.7 * green1.y) + (0.3 * self.green[1])
            g_z = (0.5 * green1.z) + (0.5 * green2.z)

            g_center = np.array([g_x, g_y, g_z])

        # If the green joint is hidden in image1 but not in image2
        elif green1.hidden and not green2.hidden:
            objects = []
            diff = []

            for obj in [red1, blue1, sphere1, box1]:
                if not obj.hidden:
                    objects.append(obj)

                    dy = self.green[1] - obj.y
                    dz = green2.z - obj.z
                    diff.append(np.linalg.norm([dy, dz]))

            if len(objects) == 0:
                rospy.logerr("Image 1 detected nothing!")
                g_y = self.green[1]
            else:
                diff_min = np.argmin(diff)
                g_y = objects[diff_min].y

            # Calculate center of green joint
            g_x = green2.x
            g_y = (0.5 * g_y) + (0.5 * self.green[1])
            g_z = (0.5 * green2.z) + (0.5 * self.green[2])
            g_center = np.array([g_x, g_y, g_z])

        # If the green joint is hidden in image2 but not in image1
        elif not green1.hidden and green2.hidden:
            objects = []
            diff = []

            for obj in [red2, blue2, sphere2, box2]:
                if not obj.hidden:
                    objects.append(obj)

                    dx = self.green[0] - obj.x
                    dz = green1.z - obj.z
                    diff.append(np.linalg.norm([dx, dz]))

            if len(objects) == 0:
                rospy.logerr("Image 2 detected nothing!")
                g_x = self.green[0]
            else:
                diff_min = np.argmin(diff)
                g_x = objects[diff_min].x

            # Calculate center of green joint
            g_x = (0.5 * g_x) + (0.5 * self.green[0])
            g_y = green1.y
            g_z = (0.5 * green1.z) + (0.5 * self.green[2])
            g_center = np.array([g_x, g_y, g_z])

        # If the green joint is completely hidden
        else:
            rospy.logwarn("Green joint is completely hidden! Using previous position...")
            g_center = self.green

        ####################
        #    Blue Joint    #
        ####################

        # If we have perfect visibility of the joint
        if not blue1.hidden and not blue2.hidden:
            b_x = (0.7 * blue2.x) + (0.3 * self.blue[0])
            b_y = (0.7 * blue1.y) + (0.3 * self.blue[1])
            b_z = (0.5 * blue1.z) + (0.5 * blue2.z)

            b_center = np.array([b_x, b_y, b_z])

        # If the blue joint is hidden in image1 but not in image2
        elif blue1.hidden and not blue2.hidden:
            objects = []
            diff = []

            for obj in [red1, green1, sphere1, box1]:
                if not obj.hidden:
                    objects.append(obj)

                    dy = self.blue[1] - obj.y
                    dz = blue2.z - obj.z
                    diff.append(np.linalg.norm([dy, dz]))

            if len(objects) == 0:
                rospy.logerr("Image 1 detected nothing!")
                b_y = self.blue[1]
            else:
                diff_min = np.argmin(diff)
                b_y = objects[diff_min].y

            # Calculate center of blue joint
            b_x = blue2.x
            b_y = (0.5 * b_y) + (0.5 * self.blue[1])
            b_z = blue2.z
            b_center = np.array([b_x, b_y, b_z])

        # If the blue joint is hidden in image2 but not in image1
        elif not blue1.hidden and blue2.hidden:
            objects = []
            diff = []

            for obj in [red2, green2, sphere2, box2]:
                if not obj.hidden:
                    objects.append(obj)

                    dx = self.blue[0] - obj.x
                    dz = blue1.z - obj.z
                    diff.append(np.linalg.norm([dx, dz]))

            if len(objects) == 0:
                rospy.logerr("Image 2 detected nothing!")
                b_x = self.blue[0]
            else:
                diff_min = np.argmin(diff)
                b_x = objects[diff_min].x

            # Calculate center of blue joint
            b_x = (0.5 * b_x) + (0.5 * self.blue[0])
            b_y = blue1.y
            b_z = blue1.z
            b_center = np.array([b_x, b_y, b_z])

        # If the blue joint is completely hidden
        else:
            rospy.logwarn("Blue joint is completely hidden! Using previous position...")
            b_center = self.blue

        #######################
        #    Target Sphere    #
        #######################

        # If we have perfect visibility of the target sphere
        if not sphere1.hidden and not sphere2.hidden:
            s_x = (0.7 * sphere2.x) + (0.3 * self.sphere[0])
            s_y = (0.7 * sphere1.y) + (0.3 * self.sphere[1])
            s_z = (0.5 * sphere1.z) + (0.5 * sphere2.z)

            s_center = np.array([s_x, s_y, s_z])

        # If the target sphere is hidden in image1 but not in image2
        elif sphere1.hidden and not sphere2.hidden:
            objects = []
            diff = []

            for obj in [red1, green1, blue1, box1]:
                if not obj.hidden:
                    objects.append(obj)

                    dy = self.sphere[1] - obj.y
                    dz = sphere2.z - obj.z
                    diff.append(np.linalg.norm([dy, dz]))

            if len(objects) == 0:
                rospy.logerr("Image 1 detected nothing!")
                s_y = self.sphere[1]
            else:
                diff_min = np.argmin(diff)
                s_y = objects[diff_min].y

            # Calculate center of target sphere
            s_x = sphere2.x
            s_y = (0.5 * s_y) + (0.5 * self.sphere[1])
            s_z = sphere2.z
            s_center = np.array([s_x, s_y, s_z])

        # If the target sphere is hidden in image2 but not in image1
        elif not sphere1.hidden and sphere2.hidden:
            objects = []
            diff = []

            for obj in [red2, green2, blue2, box2]:
                if not obj.hidden:
                    objects.append(obj)

                    dx = self.sphere[0] - obj.x
                    dz = sphere1.z - obj.z
                    diff.append(np.linalg.norm([dx, dz]))

            if len(objects) == 0:
                rospy.logerr("Image 2 detected nothing!")
                s_x = self.sphere[0]
            else:
                diff_min = np.argmin(diff)
                s_x = objects[diff_min].x

            # Calculate center of target sphere
            s_x = (0.5 * s_x) + (0.5 * self.sphere[0])
            s_y = sphere1.y
            s_z = sphere1.z
            s_center = np.array([s_x, s_y, s_z])

        # If the target sphere is completely hidden
        else:
            rospy.logwarn("Target sphere is completely hidden! Using previous position...")
            s_center = self.sphere

        ########################
        #    Adjust for bias   #
        ########################

        s_center[2] += 1

        r_center[2] += 1

        g_center[2] += 1

        #########################
        #    Publish Results    #
        #########################

        # Save newly calculated positions
        self.red = r_center
        self.green = g_center
        self.blue = b_center
        self.sphere = s_center

        # Publish positions
        self.publish(r_center, g_center, b_center, s_center)

    def publish(self, red, green, blue, sphere):
        #########################
        #    Joint Positions    #
        #########################

        msg = StateStamped()
        msg.header.stamp = rospy.Time.now()

        # Red joint position
        msg.state.red.x = red[0]
        msg.state.red.y = red[1]
        msg.state.red.z = red[2]

        # Green joint position
        msg.state.green.x = green[0]
        msg.state.green.y = green[1]
        msg.state.green.z = green[2]

        # Blue joint position
        msg.state.blue.x = blue[0]
        msg.state.blue.y = blue[1]
        msg.state.blue.z = blue[2]

        # Target position
        msg.state.sphere.x = sphere[0]
        msg.state.sphere.y = sphere[1]
        msg.state.sphere.z = sphere[2]

        self.state_pub.publish(msg)


def main():
    _ = Fusion()

    try:
        rospy.spin()
    except ROSInterruptException:
        print("Shutting down...")

#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

def distance(x1,x2,z1,z2):
    xd = x2-x1
    zd = z2-z1
    return np.sqrt(xd*xd+zd*zd)

class turtle_P(object):

    def __init__(self, pub):
        self.pub = pub

        self.input_sub = PoseStamped()
        self.mocap_sub = PoseStamped()
        self.actual = None
        self.desired = None

        self.kp_linear = .1
        self.kp_angular = .2

        self.max_ang = np.pi/20
        self.rtol = 0.2
        self.atol = 0.5

    def mocap_callback(self, msg):
        #This callback continually receives the MOCAP PoseStamped.

        #Receive the MOCAP PoseStamped
        mocap_sub = msg
        #Position (x,z)
        pos_x = mocap_sub.pose.position.x
        pos_z = mocap_sub.pose.position.z
        #Orientation (quaternion)
        q_x = mocap_sub.pose.orientation.x
        q_y = mocap_sub.pose.orientation.y
        q_z = mocap_sub.pose.orientation.z
        q_w = mocap_sub.pose.orientation.w
        quat = [q_x,q_y,q_z,q_w]
        #Quaternion to Euler orientation
        RPY = tf.transformations.euler_from_quaternion(quat)
        #output in RPY, or phi,theta,psi ******** THINK IT'S 2, MAYBE 0
        phi = RPY[2]
        #List of key things [x,z,phi]
        self.actual = [pos_x,pos_z,phi]

    def input_callback(self, msg):
        #This callback will receive the desired position, then call the
        #control function.

        #Receive the Input PoseStamped
        input_sub = msg
        #Assign x_d and z_d
        x_d = input_sub.pose.position.x
        z_d = input_sub.pose.position.z
        #List of key inputs [x_d,z_d]
        self.desired = [x_d,z_d]
        #call the calculation/publishing function
        P_control()


    def P_control(self):
        #This function will apply proportional contol and call a twist to be
        #published. It runs a loop that updates the actual position and moves
        #the turtlebot until it is (very) near that position. It will not drive
        #forward unless phi is close to phi_d.
        control_msg = Twist()
        #Initialize values of x_d,z_d,x,z for loop
        x_d = self.desired[0]
        z_d = self.desired[1]
        x = self.actual[0]
        z = self.actual[1]
        #Control Loop
        while not np.allclose([x_d,z_d],[x,z],rtol=self.rtol,atol=self.atol):
            #Receive the desired position from input_callback
            x_d = self.desired[0]
            z_d = self.desired[1]
            #Receive the actual position/orientation from mocap_callback
            x = self.actual[0]
            z = self.actual[1]
            phi = self.actual[2]
            #Calculate error in x,z Position
            x_err = x_d-x
            z_err = z_d-z
            #Calculate the desired angle from desired and actual position
            phi_d = np.arctan((x_d-x)/(z_d-z))
            if x_err > 0 and z_err < 0:
                phi_d += np.pi
            elif x_err < 0 and z_err < 0:
                phi_d -= np.pi
            #Calculate errors
            ang = phi_d-phi
            dist = distance(x_d,x,z_d,z)
            #Normalize angles
            if ang > np.pi:
                ang = np.sign(ang)*(abs(ang)-2*np.pi)
            #Implement the proportional controller; only drive forward if phi is
            #close to phi_d
            linear_speed = self.kp_linear*dist
            angular_speed = self.kp_anglar*ang
            control_msg.angular.z = angular_speed
            if ang <= self.max_ang:
                control_msg.linear.x = linear_speed
            else:
                control_msg.linear.x = 0
            #Publish control_msg
            self.pub.publish(control_msg)

        #Reset control_msg to 0
        self.pub.publish(Twist())


def main():
    rospy.init_node('turtle_P', anonymous = True)
    pub = rospy.Publisher('/my_control_topic', Twist, queue_size=10)
    turtleP = turtle_P(pub)

    while not rospy.is_shutdown():
        mocap_sub = rospy.Subscriber("", PoseStamped, turtleP.mocap_callback)
        input_sub = rospy.Subscriber("/my_input_topic", PoseStamped, turtleP.input_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

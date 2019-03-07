#!/usr/bin/env python

import rospy
import tf
import numpy as np
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def distance(x1,x2,z1,z2):
    xd = x2-x1
    zd = z2-z1
    return np.sqrt(xd*xd+zd*zd)

class turtle_P(object):

    def __init__(self, pub):
        self.pub = pub

        self.input_sub = PoseStamped()
        self.mocap_sub = Odometry()
        self.actual = None
        self.desired = None

        self.kp_linear = .1
        self.kp_angular = 0.2

        self.max_ang = np.pi/16
        self.rtol = .2
        self.atol = .5

    def mocap_callback(self, msg):
        #This callback continually receives the MOCAP PoseStamped.

        #Receive the MOCAP PoseStamped
        mocap_sub = msg
        #Position (x,z)
        pos_x = mocap_sub.pose.pose.position.x
        pos_y = mocap_sub.pose.pose.position.y
        #Orientation (quaternion)
        q_x = mocap_sub.pose.pose.orientation.x
        q_y = mocap_sub.pose.pose.orientation.y
        q_z = mocap_sub.pose.pose.orientation.z
        q_w = mocap_sub.pose.pose.orientation.w
        quat = [q_x,q_y,q_z,q_w]
        #Quaternion to Euler orientation
        RPY = tf.transformations.euler_from_quaternion(quat)
        #output in RPY, or phi,theta,psi
        phi = RPY[2]
        #List of key things [x,z,phi]
        self.actual = [pos_x,pos_y,phi]
        #print(self.actual)#for debugging

    def input_callback(self, msg):
        #This callback will receive the desired position, then call the
        #control function.

        #Receive the Input PoseStamped
        input_sub = msg
        #Assign x_d and z_d
        x_d = input_sub.pose.position.x
        y_d = input_sub.pose.position.y
        #List of key inputs [x_d,z_d]
        self.desired = [x_d,y_d]
        #call the calculation/publishing function
        self.P_control()


    def P_control(self):
        #This function will apply proportional contol and call a twist to be
        #published. It runs a loop that updates the actual position and moves
        #the turtlebot until it is (very) near that position. It will not drive
        #forward unless phi is close to phi_d.
        control_msg = Twist()
        #Initialize values of x_d,z_d,x,z for loop
        x_d = self.desired[0]
        y_d = self.desired[1]
        x = self.actual[0]
        y = self.actual[1]
        #Control Loop
        while not np.allclose([x_d,y_d],[x,y],rtol=self.rtol,atol=self.atol):
            #Receive the desired position from input_callback
            x_d = self.desired[0]
            y_d = self.desired[1]
            #Receive the actual position/orientation from mocap_callback
            x = self.actual[0]
            y = self.actual[1]
            phi = self.actual[2]
            #Calculate error in x,y Position
            x_err = x_d-x
            y_err = y_d-y
            #Calculate the desired angle from desired and actual position
            phi_d = np.arctan((x_err)/(y_err))
            if x_err > 0 and y_err < 0:
                phi_d += np.pi
            elif x_err < 0 and y_err < 0:
                phi_d -= np.pi
            #Calculate errors
            dist = distance(x_d,x,y_d,y)
            ang = phi_d-phi
            #Normalize angles
            if ang > np.pi:
                ang = np.sign(ang)*(abs(ang)-2*np.pi)
            #Implement the proportional controller; only drive forward if phi is
            #close to phi_d
            linear_speed = self.kp_linear*dist
            angular_speed = self.kp_angular*ang
            control_msg.angular.z = angular_speed
            #print("phi: "+str(phi))
            #print("phi_d: "+str(phi_d))
            print("dist: "+str(dist))
            print("ang: "+str(ang))
            print("actual: "+str(x)+" "+str(y))
            if abs(ang) <= self.max_ang:
                control_msg.linear.x = linear_speed
            else:
                control_msg.linear.x = 0
            #Publish control_msg
            self.pub.publish(control_msg)
            time.sleep(1)

        #Reset control_msg to 0
        self.pub.publish(Twist())


def main():
    rospy.init_node('turtle_P', anonymous = True)
    pub = rospy.Publisher('/my_control_topic', Twist, queue_size=10)
    #figure out what is going on here
    turtleP = turtle_P(pub)

    while not rospy.is_shutdown():
        mocap_sub = rospy.Subscriber("/odom", Odometry, turtleP.mocap_callback)
        input_sub = rospy.Subscriber("/my_input_topic", PoseStamped, turtleP.input_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

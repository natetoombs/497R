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

class turtle_Orbit(object):

    def __init__(self, pub):
        self.pub = pub

        self.input_sub = PoseStamped()
        self.mocap_sub = PoseStamped()
        self.actual = None
        self.desired = None

        self.kp_linear = 1
        self.kp_angular = 3

        self.max_ang = np.pi/6
        self.max_ang2 = np.pi/12

        self.linmax = 0.4
        self.angmax = 1.0

        self.linear_prev = 0
        self.angular_prev = 0
        self.linaccel = 0.05
        self.angaccel = 0.05

        self.Rmin = 0.05
        self.Rorbit = .8
        self.korbit = 2#np.pi/2*self.Rmin
        self.lambd = -1

        self.x_old = 0
        self.z_old = 0

    def mocap_callback(self, msg):
        #This callback continually receives the MOCAP PoseStamped.

        #Receive the MOCAP PoseStamped
        mocap_sub = msg
        #Position (x,z)
        pos_x = mocap_sub.pose.position.x
        pos_z = mocap_sub.pose.position.z
        #Orientation (quaternion)
        # q_x = mocap_sub.pose.orientation.x
        # q_y = mocap_sub.pose.orientation.y
        # q_z = mocap_sub.pose.orientation.z
        # q_w = mocap_sub.pose.orientation.w
        q_x = mocap_sub.pose.orientation.x
        q_y = -mocap_sub.pose.orientation.z
        q_z = mocap_sub.pose.orientation.y
        q_w = mocap_sub.pose.orientation.w
        quat = [q_x,q_y,q_z,q_w]
        #Quaternion to Euler orientation
        RPY = tf.transformations.euler_from_quaternion(quat)
        #output in RPY, or phi,theta,psi ******** THINK IT'S 2, MAYBE 0
        kai = RPY[2]
        #List of key things [x,z,kai]
        self.actual = [pos_x,pos_z,kai]
        #print(self.actual)


    def input_callback(self, msg):
        #This callback will receive the desired position, then call the
        #control function.

        #Receive the Input PoseStamped
        input_sub = msg
        #Using just the desired stopping point and a "starting point",
        #we will have a path, and thus a q to follow
        #Assign x_d and z_d
        x_d = input_sub.pose.position.x
        z_d = input_sub.pose.position.z
        #Assign r_x and r_z
        #r_x = #get another posestamped or something
        #r_z = #r is the path start location


        #List of key inputs [x_d,z_d]
        self.inputs = [x_d,z_d]#can include r_x,r_z
        #call the calculation/publishing function
        self.Orbit_control()


    def Orbit_control(self):
        #This function will apply proportional contol and call a twist to be
        #published. It runs a loop that updates the actual position and moves
        #the turtlebot until it is (very) near that position. It will not drive
        #forward unless phi is close to phi_d.
        control_msg = Twist()
        #Initialize values of x_d,z_d,x,z for loop
        x_d = self.inputs[0]
        z_d = self.inputs[1]
        x = self.actual[0]
        z = self.actual[1]
        #start distance above threshold
        dist = 100
        #Control Loop
        while not dist<0.3:#np.allclose([x_d,z_d],[x,z],rtol=self.rtol,atol=self.atol):
            #Receive the desired position and starting path point from
            #input_callback (in case we want to update mid "flight")
            x_d = self.inputs[0]
            z_d = self.inputs[1]

            #Receive the actual position/orientation from mocap_callback
            x = self.actual[0]
            z = self.actual[1]
            kai = self.actual[2]
            #Calculate error in x,z Position
            x_err = -(x_d-x)
            z_err = (z_d-z)#^^^^^Watch for the negative
            #Calculate phi
            phi = np.arctan(z_err/x_err)
            # if phi-kai<-np.pi:
            #     phi += 2*np.pi
            # elif phi-kai>np.pi:
            #     phi -= 2*np.pi
            if z_err > 0 and x_err < 0:
                phi += np.pi
            elif z_err < 0 and x_err < 0:
                phi -= np.pi
            dist = distance(x_d,x,z_d,z)
            #Calculate Xc, o sea, kai_d
            Xc = phi + self.lambd*(np.pi/2+np.arctan(self.korbit*((dist-self.Rorbit)/self.Rorbit)))
            #####^^^^^Not sure why, Xq and self.Xinf(at top) are negative
            ang = Xc - kai
            #Normalize angle ****************CHECK THIS
            if abs(ang) >= np.pi:
                ang = np.sign(ang)*(abs(ang)-2*np.pi)
            dist = distance(x_d,x,z_d,z)
            # print("dist: "+str(dist))
            # print("Xc: "+str(Xc))
            # print("kai: "+str(kai))
            # print("Phi: "+str(phi))
            #Implement the proportional controller; only drive forward if phi is
            #close to phi_d
            #Use a low-pass filter to make it accelerate more slowly
            linear_speed = self.kp_linear*dist
            if linear_speed > self.linmax:
                linear_speed = self.linmax
            #angular speed and max value to control_msg
            angular_speed = self.kp_angular*ang
            if angular_speed > self.angmax:
                angular_speed = self.angmax
            control_msg.angular.z = angular_speed
            #prevents the turtlebot from driving forward until it is facing close
            #to phi_d, separated by self.max_ang.
            if abs(ang) <= self.max_ang:
                if abs(ang) >= self.max_ang2:
                    control_msg.linear.x = linear_speed*(1-abs(ang))
                else:
                    control_msg.linear.x = linear_speed
            else:
                control_msg.linear.x = 0
            #record previous value of x
            self.x_old = control_msg.linear.x
            #Publish control_msg
            self.pub.publish(control_msg)

        #Reset control_msg to 0
        self.pub.publish(Twist())


def main():
    rospy.init_node('turtle_Path', anonymous = True)
    pub = rospy.Publisher('/my_control_topic', Twist, queue_size=10)
    turtleP = turtle_Orbit(pub)

    while not rospy.is_shutdown():
        mocap_sub = rospy.Subscriber("/vrpn_client_node/Raph/pose", PoseStamped, turtleP.mocap_callback)
        input_sub = rospy.Subscriber("/my_input_topic", PoseStamped, turtleP.input_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

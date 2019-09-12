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

class turtle_Path(object):

    def __init__(self, pub):
        self.pub = pub

        self.input_sub = PoseStamped()
        self.mocap_sub = PoseStamped()
        self.actual = None
        self.desired = None

        self.kp_linear = 1
        self.kp_angular = 3

        self.max_ang = np.pi/6
        self.max_ang2 = np.pi/8

        self.linmax = .5
        self.angmax = 1.2

        self.Xinf = np.pi/4
        self.kpath = 0.2

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
        phi = RPY[2]
        #List of key things [x,z,phi]
        self.actual = [pos_x,pos_z,phi]
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
        r_x = #get another posestamped or something
        r_z = #r is the path start location


        #List of key inputs [x_d,z_d]
        self.inputs = [x_d,z_d,r_x,r_z]
        #call the calculation/publishing function
        self.Path_control()


    def Path_control(self):
        #This function will apply proportional contol and call a twist to be
        #published. It runs a loop that updates the actual position and moves
        #the turtlebot until it is (very) near that position. It will not drive
        #forward unless phi is close to phi_d.
        control_msg = Twist()
        #Initialize values of x_d,z_d,x,z for loop
        x_d = self.inputs[0]
        z_d = self.inputs[1]
        #r_x = self.inputs[2]
        #r_z = self.inputs[3]
        #for determining r:
        r_zs = [4,0,-4]
        r_xs = [3.5,0,-3.5]
        if x_d > 0:
            r_x = r_xs[2]
        elif x_d == 0:
            r_x = r_xs[1]
        else:
            r_x = r_xs[0]
        if z_d > 0:
            r_z = r_zs[2]
        elif z_d == 0:
            if r_x == 0:
                r_x = r_xs[2]
            r_z = r_zs[1]
        else:
            r_z = r_zs[0]
        if r_x ==

        x = self.actual[0]
        z = self.actual[1]
        #calculate vector q as q_north, q_east
        q_n = x_d-r_x
        q_e = z_d-r_z
        #start distance above threshold
        dist = 100
        #Control Loop
        while not dist<0.05:#np.allclose([x_d,z_d],[x,z],rtol=self.rtol,atol=self.atol):
            #Receive the desired position and starting path point from
            #input_callback (in case we want to update mid "flight")
            x_d = self.inputs[0]
            z_d = self.inputs[1]
            #r_x = self.inputs[2]
            #r_z = self.inputs[3]
            q_n = x_d-r_x
            q_e = z_d-r_z
            #Receive the actual position/orientation from mocap_callback
            x = self.actual[0]
            z = self.actual[1]
            phi = self.actual[2]
            #Calculate error in x,z Position
            x_err = x_d-x
            z_err = -(z_d-z)
            # Calculate the desired angle from desired and actual position
            # phi_d = np.arctan((z_err)/(x_err))
            # if z_err > 0 and x_err < 0:
            #     phi_d += np.pi
            # elif z_err < 0 and x_err < 0:
            #     phi_d -= np.pi
            # #Calculate errors
            # ang = phi_d-phi
            #Calculate path angle
            Xq = np.atan2(q_e,q_n)
            #Calculate ep; x = north position, z = east position
            epn = x - r_x
            epe = z - r_z
            #Calculate epy
            epy = -np.sin(Xq)*epn + np.cos(Xq)*epe
            #Calculate Xc, o sea, phi_d
            Xc = Xq - Xinf*2/np.pi*np.atan(self.kpath*epy)
            ang = Xc - phi
            #Normalize angle ****************CHECK THIS
            if abs(ang) >= np.pi:
                ang = np.sign(ang)*(abs(ang)-2*np.pi)
            dist = distance(x_d,x,z_d,z)
            print("Xc: "+str(Xc))
            print(phi)
            print(dist)
            #Implement the proportional controller; only drive forward if phi is
            #close to phi_d
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
    turtleP = turtle_Path(pub)

    while not rospy.is_shutdown():
        mocap_sub = rospy.Subscriber("/vrpn_client_node/Raph/pose", PoseStamped, turtleP.mocap_callback)
        input_sub = rospy.Subscriber("/my_input_topic", PoseStamped, turtleP.input_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

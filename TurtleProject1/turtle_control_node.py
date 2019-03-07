#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def control():
    rospy.init_node('turtle_control', anonymous = True)
    control_publisher = rospy.Publisher('/my_control_topic', Twist, queue_size=10)
    control_msg = Twist()

    control_msg.linear.x = 0
    control_msg.linear.y = 0
    control_msg.linear.z = 0
    control_msg.angular.z = 0
    control_msg.angular.x = 0
    control_msg.angular.y = 0


    while not rospy.is_shutdown():
        cmd = raw_input("type 'straight' or 'turn':  ")

        if cmd == 'straight':
            speed = input("Enter your speed: ")
            distance = input("Enter your distance: ")
            control_msg.linear.x = speed
            #speed*time=distance
            current_distance = 0
            t0 = rospy.Time.now().to_sec()#current time, float
            while (current_distance < distance):
                control_publisher.publish(control_msg)
                t1 = rospy.Time.now().to_sec()
                current_distance = speed*(t1-t0)#v*dt = d

            control_msg.linear.x = 0
            control_publisher.publish(control_msg)
            control_publisher.publish(control_msg)

        if cmd == 'turn':
            speed = input("Enter your speed: ")#rad/sec
            angle = 3.14159265/180*input("Enter your turn angle (deg): ")#deg
            control_msg.angular.z = speed
            #angle = speed*time
            current_angle = 0#rad
            t0 = rospy.Time.now().to_sec()#current time, float
            while (current_angle < angle):
                control_publisher.publish(control_msg)
                t1 = rospy.Time.now().to_sec()
                current_angle = speed*(t1-t0)#v*dt = d

            control_msg.angular.z = 0
            control_publisher.publish(control_msg)
            control_publisher.publish(control_msg)

        cmd = None
        rospy.spin()

    #rospy.Subscriber("/cmd_vel_mux/active", String, callback)


if __name__ == '__main__':
        try:
            control()
        except rospy.ROSInterruptException: pass

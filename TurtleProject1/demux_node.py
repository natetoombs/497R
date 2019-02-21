#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

stop_it_now = False
demux_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def control_callback(msg):
    global stop_it_now
    reset = Twist()
    reset.linear.x=123.456
    if stop_it_now == False:
        pub_cmd(msg)


def xbox_callback(msg):
    global stop_it_now
    if msg == Twist():
        stop_it_now = False
    else:
        stop_it_now = True
        pub_cmd(msg)

def pub_cmd(msg):
    demux_pub.publish(msg)


def demux():
    rospy.init_node('demux_node', anonymous = True)
    my_control = rospy.Subscriber("/my_control_topic", Twist, control_callback)
    xbox_control = rospy.Subscriber("/teleop_velocity_smoother/raw_cmd_vel", Twist, xbox_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        demux()
    except rospy.ROSInterruptException: pass

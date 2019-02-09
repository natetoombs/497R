#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def demux():
    rospy.init_node('demux_node', anonymous = True)
    demux_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    demux_msg = Twist()

    demux_msg.linear.x = 0
    demux_msg.linear.y = 0
    demux_msg.linear.z = 0
    demux_msg.angular.z = 0
    demux_msg.angular.x = 0
    demux_msg.angular.y = 0

    while not rospy.is_shutdown():
        x = rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist)
        rospy.loginfo(x)


if __name__ == '__main__':
    try:
        demux()
    except rospy.ROSInterruptException: pass

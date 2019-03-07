#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def input_function():
    rospy.init_node('input_node', anonymous = True)
    input_pub = rospy.Publisher('my_input_topic',PoseStamped, queue_size=10)
    input_msg = PoseStamped()

    while not rospy.is_shutdown():
        input_x = input("Enter x:")
        input_z = input("Enter z:")
        input_msg.pose.position.x = input_x
        input_msg.pose.position.z = input_z

        input_pub.publish(input_msg)

if __name__ == '__main__':
        try:
            input_function()
        except rospy.ROSInterruptException: pass

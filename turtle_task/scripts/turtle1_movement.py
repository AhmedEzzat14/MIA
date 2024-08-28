#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

def key_callback(msg: Int8):
    twist = Twist()
    rospy.loginfo(f"Key received: {msg.data}")
    
    if ((msg.data == 115) | (msg.data == 83)): # S down
        twist.linear.x = -1.0
    elif ((msg.data == 119) | (msg.data == 87)): # W up
        twist.linear.x = 1.0
    elif ((msg.data == 65) | (msg.data == 97)): # A Left
        twist.angular.z = 1.0
        twist.linear.x = 1.0
    elif ((msg.data == 68) | (msg.data == 100)): # D Right
        twist.angular.z = -1.0
        twist.linear.x = 1.0
    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.0

    pub1.publish(twist)

if __name__ == '__main__':
    rospy.init_node("turtle1_movement")

    pub1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtle1/key", Int8, callback=key_callback)

    rospy.loginfo("Turtle1 movement node has started.")
    rospy.spin()

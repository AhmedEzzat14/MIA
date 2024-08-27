#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8



def key_callback(msg: Int8): 
    twist = Twist()
    rospy.loginfo(msg.data)
    
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

        pub4.publish(twist)

if __name__ == '__main__':
    rospy.init_node("turtle4_movement")
    
    pub4 = rospy.Publisher("/turtle4/cmd_vel", Twist, queue_size=0)
    sub = rospy.Subscriber("/turtle4/key", Int8, callback=key_callback)

    rospy.loginfo("Turtle4 movement node has started.")
    
    rospy.loginfo("Node has started.")
    rospy.spin()
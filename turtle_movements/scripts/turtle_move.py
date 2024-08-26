#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from time import time


last_pub_time = time()
current_publisher = 1
pub_duration = 10 

def key_callback(msg: Int8):
    global last_pub_time, current_publisher
    
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

    current_time = time()
    if (current_time - last_pub_time) >= pub_duration:
        last_pub_time = current_time
        current_publisher = (current_publisher % 4) + 1  # Cycle through 1 to 4

    if current_publisher == 1:
        pub1.publish(twist)
    elif current_publisher == 2:
        pub2.publish(twist)
    elif current_publisher == 3:
        pub3.publish(twist)
    elif current_publisher == 4:
        pub4.publish(twist)

if __name__ == '__main__':
    rospy.init_node("turtle_movement")
    
    pub1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=0)
    pub2 = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=0)
    pub3 = rospy.Publisher("/turtle3/cmd_vel", Twist, queue_size=0)
    pub4 = rospy.Publisher("/turtle4/cmd_vel", Twist, queue_size=0)
    
    sub = rospy.Subscriber("/key", Int8, callback=key_callback)
    
    rospy.loginfo("Node has started.")
    rospy.spin()

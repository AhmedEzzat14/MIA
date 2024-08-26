#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtle import Screen, Turtle
from turtlesim.msg import Pose
from std_msgs.msg import Int8


def key_callback (msg : Int8):
       # command = rospy.wait_for_message("/turtle_command", String)  # Waiting for a command from a ROS topic
        twist = Twist()
        rospy.loginfo(msg.data)
        if msg.data == 115 : # S down
            twist.linear.x = -1.0
        elif msg.data == 119 : # W up
            twist.linear.x = 1.0
        elif msg.data== 97 :  # A Left
            twist.angular.z = 1.0
            twist.linear.x = 1.0
        elif msg.data == 100 : # D Right
            twist.angular.z = -1.0
            twist.linear.x = 1.0

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node("turtle_movement")
    pub =rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=0)
    #sub = rospy.Subscriber("/turtle1/pose",Pose , callback=(pose_callback))
    sub = rospy.Subscriber("/key", Int8 , callback=(key_callback))
    rospy.loginfo("Node has started.")
    rospy.spin()
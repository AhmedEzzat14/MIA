#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry


def odom_callback(msg : Odometry ):  
    global yaw
    orientation = msg.pose.pose.orientation
    orientation_list= (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    rospy.loginfo(f"yaw : {yaw}")
    rospy.Publisher("/check_yaw" , Int8 , queue_size= 10 ).publish(yaw)
    



if __name__ == '__main__':
    rospy.init_node("yaw_checker")
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rospy.spin()
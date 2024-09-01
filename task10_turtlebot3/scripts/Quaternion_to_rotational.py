#!/usr/bin/env python3

import math
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3
# from task10_turtlebot3.msg import imuDegree
from sensor_msgs.msg import Imu


def call_back(msg : Imu ):
    global euler_in_rad
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    euler= euler_from_quaternion(quaternion)
    euler_in_rad = Vector3()
    euler_in_rad.x =euler[0]
    euler_in_rad.y =euler[1]
    euler_in_rad.z =euler[2]
    
    euler_in_degree()

def euler_in_degree ():
    global euler_in_rad
    euler_in_degree = Vector3()
    euler_in_degree.x =(euler_in_rad.x*180)/math.pi
    euler_in_degree.y =(euler_in_rad.y*180)/math.pi
    euler_in_degree.z =(euler_in_rad.z*180)/math.pi
    rospy.loginfo(f"Euler angles (roll:{euler_in_degree.x}, pitch:{euler_in_degree.y}, yaw:{euler_in_degree.z})")
    pub.publish(euler_in_degree) 

if __name__ == '__main__':
    rospy.init_node("Quaternion_to_rotational")
    pub = rospy.Publisher("/imu_in_degree" , Vector3 , queue_size= 10 )
    sub = rospy.Subscriber ("/imu" ,Imu , call_back )
    
    rospy.spin()
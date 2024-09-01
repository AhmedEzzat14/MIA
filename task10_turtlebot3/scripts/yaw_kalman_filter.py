#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

### Kalman Filter parameters
process_variance = 0.006**2  ###  based on square of the standard deviation of the gyroscope noise
error_in_measurement = 0.01  ###  matching the Gaussian noise level specified
error_in_estimate = 1.0  ### adjust it based on performance
estimate = 0.0  

def kalman_filter_update(measurement):
    global estimate, error_in_estimate

    
    prev_estimate = estimate
    prev_error_in_estimate = error_in_estimate + process_variance
    
    
    kalman_gain = prev_error_in_estimate / (prev_error_in_estimate + error_in_measurement)
    estimate = prev_estimate + (kalman_gain * (measurement - prev_estimate))
    error_in_estimate = (1 - kalman_gain) * prev_error_in_estimate

    return estimate

def imu_callback(angle: Vector3):
    yaw_angle = angle.z
    filtered_yaw = kalman_filter_update(yaw_angle)
    
    filtered_yaw_msg = Vector3()
    filtered_yaw_msg.x = angle.x
    filtered_yaw_msg.y = angle.y
    filtered_yaw_msg.z = filtered_yaw
    
    rospy.Publisher('/filtered_yaw', Vector3, queue_size=10).publish(filtered_yaw_msg)

    rospy.loginfo(f"Filtered yaw angle: {filtered_yaw_msg}")

if __name__ == '__main__':
    rospy.init_node('yaw_kalman_filter')
    
    sub = rospy.Subscriber("/imu_in_degree", Vector3, imu_callback)
    
    rospy.spin()
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

# Function to move the robot
def move_robot(pub, linear_x, linear_y, duration):
    move_cmd = Twist()
    move_cmd.linear.x = linear_x  # Forward/backward movement
    move_cmd.linear.y = linear_y  # Left/right movement
    pub.publish(move_cmd)
    rospy.sleep(duration)

    # Stop the robot after the movement
    stop_cmd = Twist()
    pub.publish(stop_cmd)
    rospy.sleep(0.2)

def robot_motion():
    # Initialize the ROS node
    rospy.init_node('finalGame', anonymous=True)

    # Publisher for velocity commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Define movement parameters
    forward_speed = 0.6  # Forward linear speed (m/s)
    backward_speed = -0.6  # Backward linear speed (m/s)
    right_speed = -0.5  # Right movement (m/s)
    left_speed = 0.5  # Left movement (m/s)

    # Time to move in each direction (seconds)
    move_duration_forward = 1.2  
    move_duration_backward = 1.1
    move_duration_sides = 0.5


    stop_duration = 0.1  # Time to stop between movements


    # Loop for continuous robot motion cycle
    while not rospy.is_shutdown():

        move_robot(pub, 0.0 , 0.0, 0.1)

        move_robot(pub, forward_speed, 0.0, move_duration_forward )
        rospy.sleep(stop_duration)


        move_robot(pub, backward_speed, 0.0, move_duration_backward)
        rospy.sleep(stop_duration)


        move_robot(pub, 0.0, right_speed, move_duration_sides)
        rospy.sleep(stop_duration)

        move_robot(pub, forward_speed, 0.0, move_duration_forward)
        rospy.sleep(stop_duration)

        move_robot(pub, backward_speed, 0.0, move_duration_backward)
        rospy.sleep(stop_duration)

        move_robot(pub, 0.0, right_speed, move_duration_sides)
        rospy.sleep(stop_duration)

        move_robot(pub, forward_speed, 0.0, move_duration_forward)
        rospy.sleep(stop_duration)

        move_robot(pub, backward_speed, 0.0,move_duration_backward)
        rospy.sleep(stop_duration)

        move_robot(pub, 0.0, left_speed, move_duration_sides)
        rospy.sleep(stop_duration)

        move_robot(pub, 0.0, left_speed, move_duration_sides)
        rospy.sleep(stop_duration)

        move_robot(pub, 0.0, left_speed, move_duration_sides)
        rospy.sleep(stop_duration)

        move_robot(pub, forward_speed, 0.0, move_duration_forward)
        rospy.sleep(stop_duration)

        move_robot(pub, backward_speed, 0.0, move_duration_backward)
        rospy.sleep(stop_duration)

        move_robot(pub, 0.0, left_speed, move_duration_sides)
        rospy.sleep(stop_duration)

        move_robot(pub, forward_speed, 0.0, move_duration_forward)
        rospy.sleep(stop_duration)

        move_robot(pub, backward_speed, 0.0, move_duration_backward)
        rospy.sleep(stop_duration)

        move_robot(pub, 0.0, right_speed, move_duration_sides)
        rospy.sleep(stop_duration)

        move_robot(pub, 0.0, right_speed, move_duration_sides)
        rospy.sleep(stop_duration)



if __name__ == '__main__':
    try:
        rospy.loginfo("Start")
        robot_motion()
    except rospy.ROSInterruptException:
        pass
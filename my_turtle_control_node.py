#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios

# Function to get key press
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def move_turtle():
    # Initialize the ROS node
    rospy.init_node('keyboard_turtle_control', anonymous=True)
    
    # Create a publisher for the turtle's velocity commands
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Create a Twist message for the velocity
    vel_msg = Twist()
    
    # Define the speed values
    linear_speed = 2.0
    angular_speed = 2.0

    print("Control Your Turtle!")
    print("W: Move forward")
    print("S: Move backward")
    print("A: Turn left")
    print("D: Turn right")
    print("Q: Quit")

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'w':
            vel_msg.linear.x = linear_speed
            vel_msg.angular.z = 0.0
        elif key == 's':
            vel_msg.linear.x = -linear_speed
            vel_msg.angular.z = 0.0
        elif key == 'a':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = angular_speed
        elif key == 'd':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -angular_speed
        elif key == 'q':
            break
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        # Publish the velocity command
        vel_pub.publish(vel_msg)

    # Stop the turtle when exiting
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass
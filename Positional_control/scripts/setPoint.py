#!/usr/bin/env python3

from geometry_msgs.msg import Twist ,Vector3
import rospy

def get_setPoint ():  # Set point 

    rospy.loginfo("Enter your setpoint X position : ")
    SP_x= float(input())
    rospy.loginfo("Enter your setpoint Y position : ")
    SP_y = float(input())
    rospy.loginfo("Enter your setpoint theta : ")
    SP_theta = float(input())
    position_SP = Twist()
    position_SP.linear = Vector3(x=SP_x, y=SP_y, z=0)
    position_SP.angular = Vector3(x=0, y=0, z=SP_theta)
    
    pub.publish(position_SP)
    rospy.loginfo(f"Published setpoint: {position_SP}")


if __name__ == '__main__':
    rospy.init_node("setPoint") 
    pub=rospy.Publisher("/setPoints",Twist ,queue_size=10)
    get_setPoint ()
    rospy.spin()
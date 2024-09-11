#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



class PID :
    def __init__(self, Kp = 0.5 ,Kd = 0.001 ,Ki = 0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = 0.0
        self.last_time = None
        self.integral_error = [0,0,0]
        self.derivative_error = [0,0,0]
        self.error= [0,0,0]
        self.error_last = [0,0,0]
        self.Process_Var = [0,0,0]
        self.position_SetPt = [0,0,0]
        self.PID_output = [0,0,0]
        self.PID_output_x = 0
        # self.setPoint = setpoint

    def setPoint_callback (self , position_SP : Twist ):  # Set point 
        rospy.loginfo("setPoint_callback")

        self.position_SetPt = [position_SP.linear.x, position_SP.linear.y, position_SP.angular.z]


    def odom_callback(self , msg : Odometry ):  #Position Variable
        position = msg.pose.pose.position
        x  = position.x
        y = position.y

        orientation = msg.pose.pose.orientation
        orientation_list= (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.Process_Var = [x,y,yaw]
        rospy.loginfo(f"yaw : {yaw}")
        rospy.loginfo(F"process_Var : {self.Process_Var}")
        rospy.loginfo(F"SETPOINTS : {self.position_SetPt}")

    def calculate_dt (self):
        current_time = rospy.Time.now()

        if self.last_time is None:
            self.last_time = current_time 
      
        self.dt = (current_time - self.last_time).to_sec()  
        self.last_time = current_time

        if self.dt == 0 or math.isnan(self.dt) or math.isinf(self.dt):
                rospy.logwarn("Invalid dt value: %f", self.dt)
                return 0.0
        self.update_PID()
        rospy.loginfo(F"dt : {self.dt}")
    def update_PID (self ):
        error_last = [0,0,0]
        

        for i in range (2) :

            # Calculate error
            self.error[i]= self.position_SetPt[i] - self.Process_Var[i]

            # Calculate the derivative error
            self.derivative_error[i] = (self.error[i]- error_last[i]) / self.dt
            error_last[i] = self.error[i]

            # Calculate the integral error
            self.integral_error[i] += self.dt * self.error[i]

            if abs(self.error[i]) >= 0.1 :                                
                self.PID_output[i] = ( self.Kp * self.error[i])+ (self.Kd * self.derivative_error[i] )+ ( self.Ki *self.integral_error[i]  )
                rospy.loginfo(f"Error: {self.error[i]}, PID Output: {self.PID_output[i]}")
            else : 
                self.PID_output[i] = 0.0
            self.update_PID_z()

    def update_PID_z (self ) :
                # Calculate error
        self.error[2]= self.position_SetPt[2] - self.Process_Var[2]

        # Calculate the derivative error
        self.derivative_error[2] = (self.error[2]- self.error_last[2]) / self.dt
        self.error_last[2] = self.error[2]

        # Calculate the integral error
        self.integral_error[2] += self.dt * self.error[2]


        if self.PID_output[0] == 0.0 and self.PID_output[1] == 0.0:
            self.PID_output[2] = ( self.Kp *self.error[2])+ (self.Kd * self.derivative_error[2] )+ ( self.Ki *self.integral_error[2]  )
            rospy.loginfo(f"PID Z : {self.PID_output[2]}")
            rospy.loginfo("yaw")
        rospy.loginfo("Calling publish_PID_output_z")

        self.publish_PID_output()

    def publish_PID_output(self):
        rospy.loginfo("publish_PID_output")

        twist = Twist()
        twist.linear.x = self.PID_output[0]  
        twist.linear.y = self.PID_output[1]  
        twist.angular.z = self.PID_output[2] 

        rospy.loginfo(f"The PID Output is : {self.PID_output}")
        rospy.Publisher('/cmd_vel', Twist, queue_size=10).publish(twist)  




  
if __name__ == '__main__':
    rospy.init_node("PID_controller")

    pid = PID()
    rospy.Subscriber("/setPoints", Twist, pid.setPoint_callback)
    rospy.Subscriber('/odom', Odometry, pid.odom_callback)
    
    rate = rospy.Rate(10)  

    while not rospy.is_shutdown():
        pid.calculate_dt()
        rate.sleep()
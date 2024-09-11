#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

position_SetPt = [0,0,0]
final_yaw = 0 # set actual yaw

class PID:
    def __init__(self, Kp= 0.6, Kd=0.001, Ki=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = 0.0
        self.last_time = None
        self.integral_error = [0, 0, 0]
        self.derivative_error = [0, 0, 0]
        self.error = [0, 0, 0]
        self.error_last = [0, 0, 0]
        self.Process_Var = [0, 0, 0]
        self.PID_output = [0, 0, 0]
        self.PID_output_x = 0
        self.PID_output_y = 0
        self.PID_output_z = 0
        self.current_state = 0  # 0 : X , 1 : Z (90 deg) , 2 : Y, 3 : Z (actual)



    def odom_callback(self, msg: Odometry):  # Position Variable
        position = msg.pose.pose.position
        x = position.x
        y = position.y

        orientation = msg.pose.pose.orientation
        orientation_list = (orientation.x, orientation.y, orientation.z, orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.Process_Var = [x, y, yaw]
        rospy.loginfo(f"yaw : {yaw}")
        rospy.loginfo(f"process_Var : {self.Process_Var}")
        rospy.loginfo(f"SETPOINTS 2 : {position_SetPt}")

    def calculate_dt(self):
        current_time = rospy.Time.now()

        if self.last_time is None:
            self.last_time = current_time

        self.dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        if self.dt == 0 or math.isnan(self.dt) or math.isinf(self.dt):
            rospy.logwarn("Invalid dt value: %f", self.dt)
            return 0.0

        self.update_PID()

    def update_error(self):
    
        for i in range (3) :

            # Calculate error
            self.error[i]= position_SetPt[i] - self.Process_Var[i]

            # Calculate the derivative error
            self.derivative_error[i] = (self.error[i]- self.error_last[i]) / self.dt
            self.error_last[i] = self.error[i]

            # Calculate the integral error
            self.integral_error[i] += self.dt * self.error[i]

    def update_PID(self):
        if position_SetPt[0] != 0.0 :
            self.update_error()

            if self.current_state == 0:  # Moving along x
                self.update_PID_x()
            elif self.current_state == 1:  # Rotating to 90 degrees
                self.update_PID_z_90()
            elif self.current_state == 2:  # Moving along y
                self.update_PID_y()
            elif self.current_state == 3:  # Rotating back to the final yaw
                self.update_PID_z_final()

    def update_PID_x(self):
        if abs(self.error[0]) >= 0.025:  
            self.PID_output[0] = (self.Kp * self.error[0]) + (self.Kd * self.derivative_error[0]) + (self.Ki * self.integral_error[0])
            self.publish_PID_output()
        else:  
            self.PID_output[0] = 0.0
            self.publish_PID_output() 
            position_SetPt[2] = (90 / 180) * math.pi  
            self.current_state = 1  
            self.update_PID()


    def update_PID_y(self):
        if abs(self.error[1]) >= 0.025:  
            self.PID_output[0] = (self.Kp * self.error[1]) + (self.Kd * self.derivative_error[1]) + (self.Ki * self.integral_error[1])
            self.publish_PID_output()
        else:  
            self.PID_output[0] = 0.0
            self.publish_PID_output()
            position_SetPt[2] = final_yaw  
            self.current_state = 3  
            self.update_PID()

    def update_PID_z_90(self):
        if abs(self.error[2]) >= 0.002: 
            self.PID_output[2] = (self.Kp * self.error[2]) + (self.Kd * self.derivative_error[2]) + (self.Ki * self.integral_error[2])
            self.publish_PID_output()
        else:  
            self.PID_output[2] = 0.0
            self.current_state = 2  
            self.update_PID()

    def update_PID_z_final(self):
        if abs(self.error[2]) >= 0.025:  
            self.PID_output[2] = (self.Kp * self.error[2]) + (self.Kd * self.derivative_error[2]) + (self.Ki * self.integral_error[2])
            self.publish_PID_output()
        else:  
            self.PID_output[2] = 0.0
            self.publish_PID_output()

    def regulating_PID_output(self):

        if ( self.PID_output[2]>= 1.82 ):
            self.PID_output[2] = 1.7
        if (self.PID_output[0]>= 0.26 ):
            self.PID_output[0] = 0.20
        

    def publish_PID_output(self):
        self.regulating_PID_output()
        rospy.loginfo("publish_PID_output")

        twist = Twist()
        twist.linear.x = self.PID_output[0]
        twist.angular.z = self.PID_output[2]

        rospy.loginfo(f"The PID Output is : {self.PID_output}")
        pub.publish(twist)

def setPoint_callback(position_SP: Twist):  # Set point
    global position_SetPt 
    global final_yaw

    rospy.loginfo("setPoint_callback")
    position_SetPt = [position_SP.linear.x, position_SP.linear.y, position_SP.angular.z]
    rospy.loginfo(f"SETPOINTS 1 : {position_SetPt}")
    final_yaw = position_SP.angular.z  

if __name__ == '__main__':
    global pub 
    rospy.init_node("PID_controller")
    rospy.loginfo(f"SETPOINTS 3 : {position_SetPt}")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/setPoints", Twist, setPoint_callback)

    pid = PID()
    
    rospy.Subscriber('/odom', Odometry, pid.odom_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pid.calculate_dt()
        rate.sleep()

#  TASK10.3- Keep me updated

## plan : 
1. Gazebo-Installation.
2. getting ready for TurtleBot.
3. Setup.
4. Launching your first Gazebo world.
5. using keyboard to move the TurtleBot3.
6. Install rviz.
7. visualize the data in RViz.
8. Read IMU data from the turtlebot.
9. visualize the LiDAR data on RVIZ.
10. Convert the IMU readings from Quaternion to Degree and publish them on new topic.
11. Add gaussian noise to the IMU data from the configuration files (.urdf).
12. Closer look to 1D-kalman Filter.
13. Implement 1D-Kalman Filter on YAW angle from the IMU by fusing.
previous YAW angles with latest YAW angle.
14. Visualize the filtered data after Kalman filter and the noisy data on
rqt_multiplot.

## porcoess :
### 1.Installation 
[Guide on how to install gezebo]( https://github.com/Abdalla-El-gohary/Gazebo-Installation/tree/main )

### 2.getting ready for TurtleBot
1. Install the core dependencies of TurtleBot

* ```sudo apt install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan \
  ros-noetic-rosserial-arduino ros-noetic-rosserial-python \
  ros-noetic-rosserial-server ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
  ros-noetic-gazebo-ros-pkgs```


2. Install the package for the servo actuators (motors) used on TurtleBot
* ``` sudo apt install ros-noetic-dynamixel-sdk```
3. Install  package that defines the messages that TurtleBot uses to communicate with ROS:
* ``` sudo apt install ros-noetic-turtlebot3-msgs```
4. Install the main TurtleBot3 package:
* ```sudo apt install ros-noetic-turtlebot3```
5. Install the TurtleBot3 Simulations package from source, by cloning it with Git into our Catkin Workspace:
* ```cd ~/catkin_ws/src/```
*  ```git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git```
6. Navigate back to the root of the Catkin Workspace to run a catkin_make
* ```cd ~/catkin_ws```
* ```catkin_make```

### 3. Setup
1. edit your ~/.bashrc
* ```echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc```
* ```echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc```
2. open your ~/. bashrc and check  
*  ```gedit ~/.bashrc```
3. if the sources is not written if not write it manually
* ```source /opt/ros/noetic/setup.bash```
* ```source ~/catkin_ws/devel/setup.bash```
* ```export TURTLEBOT3_MODEL=burger```

### 4. Launching your first Gazebo world
*  ```roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch```
* ```gz physics -s 0.01```

### 5.using keyboard to move the TurtleBot3
* ```roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch ```

### 6.Install rviz
* ```sudo apt-get install ros-fuerte-visualization```
* ```sudo apt-get install ros-noetic-rviz```
* ```source /opt/ros/indigo/setup.bash```
* ```roscore```

### 7. visualize the data in RViz
* ```roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch```

### 8. Read IMU data from the turtlebot
* use ```rostopic list``` to search for the topic the publish the imu 
* ```rostopic echo /imu```

### 9. visualize the LiDAR data on RVIZ
* ```roslaunch turtlebot3_gazebo turtlebot3_house.launch``` to have some obstacles near you.
* ```roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch```
* press add at the lower left then laserScan then Ok.
* topic by defualt will be /scan if not add it. 
* Now you can visualize it.
* use ```rostopic echo /scan``` to see what does the LiDAR's topic publish.

### 10.  Convert the IMU readings from Quaternion to Degree and publish them on new topic
* Create your package :
   * catkin_create_pkg <package_name> rospy roscpp tf geometry_msgs sensor_msgs ```catkin_create_pkg task10_turtlebot3 rospy roscpp tf geometry_msgs sensor_msgs```
   * back to catkin_ws cd catkin_ws
   * ```catkin_make``` 
   * ```cd build/``` you will find package_name
   * cd to your package the ```mkdir scripts```
   * touch Quaternion_to_rotational.py
   * chmod +x Quaternion_to_rotational.py
* download package that deals with transformition :
   * ```sudo apt-get install ros-noetic-tf```
* Write the code of the conversition node : you will find it in the package 
   
### 11. Add gaussian noise to the IMU data from the configuration files (.urdf)
* ```cd /opt/ros/noetic/share/turtlebot3_description```
* navigatoe to imu_link
* change ```<gaussianNoise>0.0</gaussianNoise>``` to ```<gaussianNoise>0.01</gaussianNoise>```
* Adding guassian Noise to angular velocity :
  * Gyroscope Noise for Standard Deviation : change ```<stddev>2e-4</stddev>``` to ```<stddev>0.006</stddev> ```
  * Gyroscope bias noise : change ```<bias_stddev>0.0000008</bias_stddev>``` to ```<bias_stddev>0.0005</bias_stddev>```
* Adding guassian Noise to angular velocity :
  * Accelerometer Noise for Standard Deviation : change ```<stddev>1.7e-2</stddev>``` to  ```<stddev>0.006</stddev>``` 
  * Accelerometer bias noise : change ```<bias_stddev>0.001</bias_stddev>``` to ```<bias_stddev>0.005</bias_stddev>```

### 12. Closer look to 1D-kalman Filter
* abbreviations :
  * KALMAN Gain = KG
  * E(Est) = error in estimate
  * E(MEA) = error in measurment
  * EST(t) = Current(new) estimate
  * Est(t-1) = previous estimate
  * MEA = measurment
  * E(Est)(t) = Error in Current(new) estimate 
  * E(Est)(t-1) = Error in previous estimate
* equation and the explanition :
  * KG = E(Est)/( E(Est)+E(MEA) )
  * EST(t)=EST(t-1)+KG[ MEA -EST(t-1) ]
  * E(Est)(t)=[ 1 - KG ][E(Est)(t-1)]
* explaination : 
  * KG puts more weight on either measurment or estimate
  * 0<=KG<=1
  * if KG is large then the error in measurment relative to error in estimate is small so in KG[ MEA -EST(t-1) ] KG only takes low portion of [ MEA -EST(t-1) ] as it trusts the measured values more than estimate 
  * if KG is small then the error in estimate relative to error in  measurment is small so in KG[ MEA -EST(t-1) ] KG doesnt give weight to [ MEA -EST(t-1) ] as it doesnt trust the measured values and trusts estimate more
  * [ 1 - KG ] is considerd inverse size of KG
  * if KG is large then [ 1 - KG ] is small so the E(Est)(t) will take only small portion of [E(Est)(t-1)] as it trusts the measurement more 
  * if KG is small then [ 1 - KG ] is large so the E(Est)(t) will take only large portion of [E(Est)(t-1)] as it trusts the estimate more 
  * if KG is large the Kg get near to actual values quickly 
  * if KG is small the Kg get near to actual values slowly 

### 13. Implement 1D-Kalman Filter on YAW angle from the IMU by fusing previous YAW angles with latest YAW angle
* go to the scripts in the package 
* ``` touch yaw_kalman_filter.py```
* ``` chmod +x yaw_kalman_filter.py```
* Write the code : you will find it in the package

### 14. Visualize the filtered data after Kalman filter and the noisy data on
rqt_multiplot 
  * open multiplot ```rosrun rqt_multiplot rqt_multiplot```
  * configure new plot for the filterd yaw y-axis /filterd_yaw , x-axis (message receipt time) 
  * configure another plot for the noisy yaw y-axis /imu_in_degree , x-axis (message receipt time) 



## Refrences :
* https://github.com/sundevilrobotics/urc-code/wiki/4.-Learning-ROS-through-Simulation
* https://youtube.com/playlist?list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT&si=Pw-hYwMVvnHjzcPM

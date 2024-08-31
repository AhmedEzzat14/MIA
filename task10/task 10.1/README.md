# Day one of TASK10.3- Keep me updated

## plan : 
1. Gazebo-Installation
2. getting ready for TurtleBot
3. Setup
4. Launching your first Gazebo world
4. using keyboard to move the TurtleBot3

## porcoess :
### 1.Installation 
[Guide on how to install gezebo]( https://github.com/Abdalla-El-gohary/Gazebo-Installation/tree/main )

### 2.getting ready for TurtleBot
1. Install the core dependencies of TurtleBot

* > sudo apt install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers \
  ros-melodic-gazebo-ros-pkgs


2. Install the package for the servo actuators (motors) used on TurtleBot
* > sudo apt install ros-noetic-dynamixel-sdk
3. Install  package that defines the messages that TurtleBot uses to communicate with ROS:
* > sudo apt install ros-noetic-turtlebot3-msgs
4. Install the main TurtleBot3 package:
* > sudo apt install ros-noetic-turtlebot3
5. Install the TurtleBot3 Simulations package from source, by cloning it with Git into our Catkin Workspace:
* >cd ~/catkin_ws/src/
*  >git clone -b melodic-devel https://<!--This is a comment-->github.com/<!--This is, too-->ROBOTIS-GIT/turtlebot3_simulations.git
6. Navigate back to the root of the Catkin Workspace to run a catkin_make:
* >cd ~/catkin_ws
* > catkin_make

### 3. Setup
1. edit your ~/.bashrc
* echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
* echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
2. open your ~/. bashrc and check  
* > gedit ~/.bashrc
3. if the sources is not written if not write it manually
* source /opt/ros/noetic/setup.bash
* source ~/catkin_ws/devel/setup.bash
* export TURTLEBOT3_MODEL=burger

### 4. Launching your first Gazebo world
* > roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
* > gz physics -s 0.01

### 5.using keyboard to move the TurtleBot3
* >roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 


## Refrences :
* https://github.com/sundevilrobotics/urc-code/wiki/4.-Learning-ROS-through-Simulation
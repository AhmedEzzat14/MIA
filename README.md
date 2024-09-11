# TASK12.1- Positional Control:shato:
## Aim:
  1-TurtleBot PID positional control using gazebo
  2-Demonstrate how this can be extended to a Shato robot & simulating its control model in a similar environment
  
## process
1-TurtleBot PID positional control using gazebo
  - setPoint node for choosing our setpoints
  - turtlebot3_PID node which include PID logic &  we tune the PID parameters
  - Since the robot uses differential drive, we can't move in linear x and linear y at the same time so we move in linear x till we reach the setPont x >> rotate 90 degree >> move in linear x till reach setPont y >> move in angular z till reach setPoint yaw

  2- Demonstrate how this can be extended to a Shato robot & simulating its control model in a similar environment
  - Go to shato package  >>  create Shato_gazebo package >> create launch directory >> create launch file
  - use same setPoint node for choosing our setpoints
  - Shato_PID node which include PID logic &  we tune the PID parameters
  - Shato can move in x , y at the same time but the velcities are in local frame so chaning the angular z with time will change the axis x and y >> move in linear x and y at the same time till reach setPoints x and y >> move in angular z till reach setPoint yaw

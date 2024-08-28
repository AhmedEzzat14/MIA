#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from turtlesim.srv import Kill, KillRequest

def turtle_kill_callback(msg):
    global turtle_name
    turtle_name = msg.data
    rospy.loginfo(f"Received request to kill turtle: {turtle_name}")

    # Wait for the kill service to be available
    rospy.wait_for_service('/kill')
    try:
        # Create a service proxy to the /kill service
        kill_turtle_service = rospy.ServiceProxy('/kill', Kill)
        
        # Call the service with the turtle name
        kill_request = KillRequest(name=turtle_name)
        kill_turtle_service(kill_request)
        rospy.loginfo(f"Turtle '{turtle_name}' has been killed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def kill_node():
    rospy.init_node('kill_node', anonymous=True)
    
    # Subscribe to the topic that publishes the turtle name to be killed
    turtle_health_topic = f"/{turtle_name}/health_status"
    rospy.Subscriber(turtle_health_topic, String, turtle_kill_callback)
    
    rospy.loginfo("kill_node is ready and waiting for turtle names to kill.")
    
    # Keep the node running until it's shut down
    rospy.spin()

if __name__ == '__main__':
    kill_node()

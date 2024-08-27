#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn

def spawn_turtle(x, y, theta, name):
    rospy.wait_for_service('/spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(x, y, theta, name)
        rospy.loginfo(f"Turtle '{name}' spawned at ({x}, {y}) with theta={theta}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('spawn_turtles')

    # Coordinates and names for the turtles
    turtles = [
        {'x': 2.0, 'y': 2.0, 'theta': 0.0, 'name': 'turtle1'},
        {'x': 2.0, 'y': 2.0, 'theta': 0.0, 'name': 'turtle2'},
        {'x': 8.0, 'y': 4.0, 'theta': 0.0, 'name': 'turtle3'},
        {'x': 1.0, 'y': 8.0, 'theta': 0.0, 'name': 'turtle4'}
    ]

    for turtle in turtles:
        spawn_turtle(turtle['x'], turtle['y'], turtle['theta'], turtle['name'])

    rospy.loginfo("All turtles have been spawned.")

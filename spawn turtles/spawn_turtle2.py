#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn
import pygame 

pygame.mixer.init()
pygame.mixer.music.load('sound.mp3')  
pygame.mixer.music.play(-1) 

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
        {'x': 2.0, 'y': 2.0, 'theta': 0.0, 'name': 'turtle2'}
    ]

    for turtle in turtles:
        spawn_turtle(turtle['x'], turtle['y'], turtle['theta'], turtle['name'])

    rospy.loginfo("All turtles have been spawned.")
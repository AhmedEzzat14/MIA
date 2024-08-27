#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill, SetPen
import random
import math
import time
from std_msgs.msg import String

# Game parameters
attack_radius = 3.0  # Attack radius in turtle units
attack_limit = 10
turn_time_limit = 30  # Time limit for each turtle's turn in seconds

class Turtle:
    def __init__(self, name, x, y):
        self.name = name
        self.attack_count = 0

        # Create the turtle in turtlesim
        self.spawn_turtle(name, x, y)

    def spawn_turtle(self, name, x, y):
        rospy.wait_for_service('spawn')
        try:
            spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
            spawn_turtle(x, y, random.uniform(0, 2 * math.pi), name)
            rospy.loginfo(f"Turtle {name} spawned at ({x}, {y}).")
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service call failed: {e}")

    def attack(self):
        if self.attack_count >= attack_limit:
            rospy.loginfo(f"Turtle {self.name} has reached the attack limit.")
            return
        
        self.attack_count += 1
        pub = rospy.Publisher('attack', String, queue_size=10)
        attack_msg = f"{self.name} attacks!"
        pub.publish(attack_msg)
        rospy.loginfo(f"{self.name} performed an attack.")

def main():
    rospy.init_node('turtle_attack_game')

    # Create the turtle
    turtle_name = 'turtle1'
    t = Turtle(turtle_name, -8, 8)

    rospy.loginfo(f"It's {turtle_name}'s turn.")
    rospy.loginfo(f"You have {turn_time_limit} seconds to attack.")

    # Wait for a specified time and then perform an attack
    time.sleep(turn_time_limit)
    t.attack()

    rospy.loginfo(f"{turtle_name} performed an attack.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

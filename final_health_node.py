#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import String
import math

# Game parameters
attack_radius = 3.0  # Attack radius in turtle units
damage = 50

class TurtleHealthManager:
    def __init__(self):
        self.turtles = {}
        self.sub_pose = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.sub_attack = rospy.Subscriber('attack', String, self.handle_attack)

        # Add turtles
        self.add_turtle('turtle1', -8, 8)
        self.add_turtle('turtle2', 2, 2)
        self.add_turtle('turtle3', 8, -8)
        self.add_turtle('turtle4', -2, -2)

    def add_turtle(self, name, x, y):
        self.turtles[name] = {'pose': (x, y), 'health': 100}

    def update_pose(self, msg):
        # Update pose of the turtle
        turtle_name = rospy.get_name().split('/')[-1]
        self.turtles[turtle_name]['pose'] = (msg.x, msg.y)

    def handle_attack(self, msg):
        attacking_turtle = msg.data.split()[0]
        rospy.loginfo(f"Received attack message from {attacking_turtle}")

        for name, data in self.turtles.items():
            if name != attacking_turtle:
                dist = self.get_distance(attacking_turtle, name)
                if dist <= attack_radius:
                    self.turtles[name]['health'] -= damage
                    rospy.loginfo(f"Turtle {name}'s health is now {self.turtles[name]['health']}.")
                    if self.turtles[name]['health'] <= 0:
                        rospy.loginfo(f"Turtle {name} has been killed.")
                        del self.turtles[name]

    def get_distance(self, attacker_name, target_name):
        attacker_pose = self.turtles[attacker_name]['pose']
        target_pose = self.turtles[target_name]['pose']
        return math.sqrt((attacker_pose[0] - target_pose[0]) ** 2 + (attacker_pose[1] - target_pose[1]) ** 2)

def main():
    rospy.init_node('turtle_health_manager')
    TurtleHealthManager()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

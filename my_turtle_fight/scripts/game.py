#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill, SetPen
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math
import sys
import select
import termios
import tty
import time

# Game parameters
attack_radius = 3.0  # Attack radius in turtle units
initial_health = 100
num_turtles = 4
attack_limit = 10
turn_time_limit = 30  # Time limit for each turtle's turn in seconds

# Turtle positions and colors
turtle_positions = [(-8, 8), (8, 8), (2, 6), (1, 4)]
turtle_colors = ['blue', 'green', 'orange', 'purple']
turtles = []

class Turtle:
    def __init__(self, name, x, y, color):
        self.name = name
        self.health = initial_health
        self.attack_count = 0

        # Create the turtle in turtlesim
        self.spawn_turtle(name, x, y)

        # Set turtle color
        rospy.wait_for_service(f'/{name}/set_pen')
        try:
            set_pen = rospy.ServiceProxy(f'/{name}/set_pen', SetPen)
            set_pen(255, 255, 255, 2, 0)  # White pen for drawing
        except rospy.ServiceException as e:
            rospy.logerr(f"SetPen service call failed: {e}")

    def spawn_turtle(self, name, x, y):
        rospy.wait_for_service('spawn')
        try:
            spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
            spawn_turtle(x, y, random.uniform(0, 2 * math.pi), name)
            rospy.loginfo(f"Turtle {name} spawned at ({x}, {y}).")
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service call failed: {e}")

    def kill_turtle(self):
        rospy.wait_for_service('kill')
        try:
            kill_turtle = rospy.ServiceProxy('kill', Kill)
            kill_turtle(self.name)
            rospy.loginfo(f"Turtle {self.name} has been killed.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Kill service call failed: {e}")

    def move(self, linear_x, angular_z):
        pub = rospy.Publisher(f'/{self.name}/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        pub.publish(twist)
        rate.sleep()

    def stop(self):
        self.move(0, 0)

    def attack(self, targets):
        if self.attack_count >= attack_limit:
            rospy.loginfo(f"Turtle {self.name} has reached the attack limit.")
            return

        self.attack_count += 1
        attacked = False

        for target in targets:
            if target.name != self.name:
                dist = self.get_distance(target)
                if dist <= attack_radius:
                    rospy.loginfo(f"Turtle {self.name} attacks {target.name}!")
                    target.health -= 10
                    rospy.loginfo(f"{target.name}'s health is now {target.health}.")

                    # Kill the target if health is zero or less
                    if target.health <= 0:
                        target.kill_turtle()
                        targets.remove(target)
                        rospy.loginfo(f"{target.name} has been removed from the game.")
                    attacked = True
                    break  # End the turn after one attack

        if not attacked:
            rospy.loginfo(f"Turtle {self.name} did not find any targets within attack radius.")

    def get_distance(self, target):
        pose = rospy.wait_for_message(f'/{self.name}/pose', Pose)
        target_pose = rospy.wait_for_message(f'/{target.name}/pose', Pose)
        return math.sqrt((pose.x - target_pose.x) ** 2 + (pose.y - target_pose.y) ** 2)

def determine_winner(turtles):
    alive_turtles = [t for t in turtles if t.health > 0]
    if alive_turtles:
        winner = max(alive_turtles, key=lambda t: t.health)
        rospy.loginfo(f"The winner is {winner.name} with {winner.health} health remaining!")
    else:
        rospy.loginfo("No winner, all turtles are dead.")

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key

def main():
    rospy.init_node('turtle_attack_game')

    # Create turtles
    for i in range(num_turtles):
        name = f'turtle{i+1}'
        t = Turtle(name, turtle_positions[i][0], turtle_positions[i][1], turtle_colors[i])
        turtles.append(t)

    turn_index = 0

    while not rospy.is_shutdown():
        if not any(t.health > 0 for t in turtles):
            break

        # Display which turtle's turn it is
        current_turtle = turtles[turn_index]
        rospy.loginfo(f"It's {current_turtle.name}'s turn.")

        # Manual control
        rospy.loginfo(f"Use arrow keys to move the turtle. Press 'q' to attack. You have {turn_time_limit} seconds.")

        start_time = time.time()
        turn_active = True

        while turn_active and (time.time() - start_time) < turn_time_limit:
            key = get_key()
            linear_x = 0
            angular_z = 0

            if key == '\x1b':  # Arrow keys
                key = get_key()
                if key == '[':
                    key = get_key()
                    if key == 'A':  # Up arrow
                        linear_x = 1.0
                    elif key == 'B':  # Down arrow
                        linear_x = -1.0
                    elif key == 'C':  # Right arrow
                        angular_z = -1.0
                    elif key == 'D':  # Left arrow
                        angular_z = 1.0

            elif key == 'q' or key == 'Q':
                current_turtle.attack(turtles)
                current_turtle.stop()
                turn_active = False
                break  # End the turn after one attack

            current_turtle.move(linear_x, angular_z)

        if turn_active:
            # Penalize the turtle for not acting in time
            rospy.loginfo(f"Turtle {current_turtle.name} failed to act in time!")
            current_turtle.health -= 10
            current_turtle.attack_count = max(0, current_turtle.attack_count - 1)
            rospy.loginfo(f"{current_turtle.name}'s health is now {current_turtle.health}. Attack count decreased to {current_turtle.attack_count}.")

        # Check if all turtles have reached the attack limit
        if all(t.attack_count >= attack_limit for t in turtles):
            break

        # Move to the next turtle
        turn_index = (turn_index + 1) % num_turtles

    # Determine the winner
    determine_winner(turtles)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from turtlesim.srv import TeleportAbsolute
from std_msgs.msg import String

def choose_turtle():
    rospy.init_node('turtle_selector')

    # Create a publisher for the turtle name
    turtle_pub = rospy.Publisher('/chosen/{current_turtle}', String, queue_size=10, latch=True)  

    # List of available turtles
    turtles = ["turtle1", "turtle2", "turtle3", "turtle4"]

    print("Available turtles:")
    for i, turtle in enumerate(turtles, 1):
        print(f"{i}. {turtle}")

    # Ask the user to choose a turtle
    choice = int(input("Enter the number of the turtle you want to choose: "))

    if 1 <= choice <= len(turtles):
        selected_turtle = turtles[choice - 1]
        print(f"You chose: {selected_turtle}")

        # Publish the selected turtle's name
        turtle_pub.publish(selected_turtle)

    else:
        print("Invalid choice. Please choose a number between 1 and 4.")

if __name__ == '__main__':
    try:
        choose_turtle()
    except rospy.ROSInterruptException:
        pass

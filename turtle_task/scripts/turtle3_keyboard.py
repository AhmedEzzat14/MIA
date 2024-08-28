#!/usr/bin/env python3
import getch
import rospy
from std_msgs.msg import String, Int8

current_turtle = "turtle3"  # Default turtle

def turtle_callback(msg):
    global current_turtle
    current_turtle = msg.data
    rospy.loginfo(f"Current turtle set to: {current_turtle}")

def keys():
    global current_turtle
    rospy.init_node('keyboard_node', anonymous=True)

    # Initialize publisher for dynamic turtle topic
    pub = rospy.Publisher('/turtle3_key', Int8, queue_size=10, latch=True)

    rospy.Subscriber('/chosen/turtle3', String, turtle_callback)

    rospy.sleep(1)
    rate = rospy.Rate(100)  # Adjust the rate as needed
    while not rospy.is_shutdown():
        k = ord(getch.getch())  # Convert the keypress event to an ord value
        if k in [119, 115, 100, 97, 113, 65, 87, 83, 68, 81]:  # Filtered keys
            rospy.loginfo(f"Key pressed: {k}")

            # Publish the key press to the appropriate turtle movement node   
            rospy.Publisher("/turtle3/key", Int8, queue_size=10, latch=True).publish(k)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        rospy.loginfo("turtle3_keyboard terminated")

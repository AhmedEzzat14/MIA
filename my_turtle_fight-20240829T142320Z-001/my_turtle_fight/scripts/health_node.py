#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int8
from turtlesim.srv import Kill



turtles = ['turtle1','turtle2','turtle3','turtle4']

attacked_turtle = None
attacker_turtle = None

# turtle1_health = 100
# turtle2_health = 100 
# turtle3_health = 100 
# turtle4_health = 100

health = {
    'turtle1': 100,
    'turtle2': 100,
    'turtle3': 100,
    'turtle4': 100
}

# rospy.init_node('health_manager', anonymous=True)

health_status = "Healthy"
health_level = 100.0 

def attacked_callback(msg : String ):
    global attacked_turtle
    attacked_turtle = msg.data
    rospy.loginfo(f"attacked_turtle: {attacked_turtle}")

def attacker_callback(msg : String):
    global attacker_turtle
    attacker_turtle = msg.data
    rospy.loginfo(f"attacker_turtle: {attacker_turtle}")



# def health_callback(msg: Int8, attacked_tur: str):
def health_callback(msg: Int8):
    global health
    health[attacked_turtle] = msg.data


if __name__ == '__main__':

    rospy.init_node("health_node", anonymous=True)

    rospy.Subscriber("/attacked", String , attacked_callback)
    rospy.Subscriber("/attacker", String , attacker_callback)

    for attacked_tur in turtles:
        # rospy.Subscriber(f"/{attacked_tur}/health", Int8, health_callback, attacked_tur)
        rospy.Subscriber(f"/{attacked_tur}/health", Int8, health_callback)


    rospy.wait_for_service('/kill')
    kill_service = rospy.ServiceProxy('/kill', Kill)


    # rospy.spin ()
    try:
        while not rospy.is_shutdown():
            if attacked_turtle is not None:
                if attacked_turtle in health:
                    rospy.loginfo(f"health of {attacked_tur} is {health[attacked_turtle]}")
                    if health[attacked_turtle] == 0:
                        health_status = "died"
                        rospy.loginfo(f"{attacked_turtle} died")
                        try:
                            response = kill_service(attacked_turtle)
                            rospy.loginfo(f"{attacked_turtle} killed successfully")
                        except rospy.ServiceException as e:
                            rospy.logwarn(f"Service call failed: {e}")

            rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass
    






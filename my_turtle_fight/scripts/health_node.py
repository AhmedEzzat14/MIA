#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int8
from turtlesim.srv import Kill



turtles = ['turtle1','turtle2','turtle3','turtle4']

attacked_turtle = 'turtle1'
attacker_turtle = 'turtle2'

turtle1_health = 100
turtle2_health = 100 
turtle3_health = 100 
turtle4_health = 100

health = {
    'turtle1': 100,
    'turtle2': 100,
    'turtle3': 100,
    'turtle4': 100
}

rospy.init_node('health_manager', anonymous=True)

health_status = "Healthy"
health_level = 100.0 

def attacked_callback(msg : String ):
    global attacked_turtle
    attacked_turtle = msg

def attacker_callback(msg : String):
    global attacker_turtle
    attacker_turtle = msg



def health_callback(msg: Int8, attacked_tur: str):
    global health
    health[attacked_tur] = msg.data

rospy.Subscriber("/attacked", String , (attacked_callback))

for attacked_tur in turtles:
    rospy.Subscriber(f"/{attacked_tur}/damage", Int8, health_callback, attacked_tur)

rospy.Subscriber("/attacker", String , attacker_callback)

rospy.wait_for_service('/kill')
kill_service = rospy.ServiceProxy('/kill', Kill)






if __name__ == '__main__':
    # try:
    #     while not rospy.is_shutdown():

            for other_turtle in turtles :
                    if other_turtle != attacker_turtle:
                        if health[other_turtle] < 20.0:
                            health_status = "Low Health"
                            rospy.loginfo(f"Turtle {attacked_turtle} has low health")

                        elif health[other_turtle] <= 0:
                            health_status = "died"
                            try:
                                response = kill_service(attacked_turtle)
                                if response.success:
                                    rospy.loginfo(f"{attacked_turtle} killed successfully")
                            except rospy.ServiceException as e:
                                rospy.logwarn(f"Service call failed: {e}")

                            health_level = 0
                        else:
                            health_status = "Healthy"
                            rospy.loginfo(f"{attacked_turtle} is healthy ")
            rospy.spin()
    #     rospy.sleep(1)  # Sleep for 1 second before the next check
    # except rospy.ROSInterruptException:
    #     pass
    







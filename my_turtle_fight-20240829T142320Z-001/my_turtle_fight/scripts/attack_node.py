#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int8 , String
from turtlesim.msg import Pose


attack_limit = 10
damage = 50
#current_turtle = "turtle1"

turtle_name = rospy.get_param('~turtle_name', 'turtle1')

turtle_positions = {
    'turtle1': {'x': 0, 'y': 0},
    'turtle2': {'x': 0, 'y': 0},
    'turtle3': {'x': 0, 'y': 0},
    'turtle4': {'x': 0, 'y': 0}
}

signal_of_attack = None

health = {
    'turtle1': 100,
    'turtle2': 100,
    'turtle3': 100,
    'turtle4': 100
}

attack_count = {
    'turtle1': 0,
    'turtle2': 0,
    'turtle3': 0,
    'turtle4': 0
}
#msg = Pose()



def Q_callback(msg : Int8):
    global signal_of_attack
    signal_of_attack = msg.data
    



def turtle_pose_callback(turtle_name, pose):
    global signal_of_attack
    global turtle_positions
    turtle_positions[turtle_name]['x'] = pose.x
    turtle_positions[turtle_name]['y'] = pose.y

    
    if(signal_of_attack == 113 or signal_of_attack == 81 ):
            for other_turtle in turtle_positions:
                if health[other_turtle] > 0 :
                    if other_turtle != turtle_name:
                        if ((pose.x <= turtle_positions[other_turtle]['x'] + 1 or pose.x <= turtle_positions[other_turtle]['x']- 1 ) and  
                            (pose.y <= turtle_positions[other_turtle]['y'] or pose.y <= turtle_positions[other_turtle]['y'] - 1 )):
                            health[other_turtle] = health[other_turtle]- damage
                            attack()
                            pub1.publish(health[other_turtle])
                            pub2.publish(other_turtle)
                            pub3.publish(turtle_name)
                            rospy.loginfo(f"{other_turtle}  in {turtle_name} zone ")
                            rospy.loginfo(f"{turtle_name} performed an attack on {other_turtle}.")
                            
            signal_of_attack = 0



def attack():
    global attack_count
    if attack_count[turtle_name] >= attack_limit:
        rospy.loginfo(f"{turtle_name} has reached the attack limit.")
        
    else :
     attack_count[turtle_name] += 1
     rospy.loginfo(f"{turtle_name} attacked {attack_count[turtle_name]}")

    

if __name__ == '__main__':
    rospy.init_node('turtle_attack_game')

    turtle_attack_topic = f"/{turtle_name}/attack_time" 
    turtle_damage_topic = f"/{turtle_name}/health"

    pub1 = rospy.Publisher(turtle_damage_topic, Int8, queue_size=10)
    pub2 = rospy.Publisher("/attacked", String , queue_size= 10)
    pub3 = rospy.Publisher("/attacker", String , queue_size= 10 )

    sub = rospy.Subscriber(turtle_attack_topic, Int8, callback=Q_callback)

    for turtle in turtle_positions:
        rospy.Subscriber(f"/{turtle}/pose", Pose, lambda msg, t=turtle: turtle_pose_callback(t, msg))


    # rospy.Subscriber("/turtle1/pose", Pose, lambda msg: turtle_pose_callback('turtle1', msg))
    # rospy.Subscriber("/turtle2/pose", Pose, lambda msg: turtle_pose_callback('turtle2', msg))
    # rospy.Subscriber("/turtle3/pose", Pose, lambda msg: turtle_pose_callback('turtle3', msg))
    # rospy.Subscriber("/turtle4/pose", Pose, lambda msg: turtle_pose_callback('turtle4', msg))

    rospy.spin()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

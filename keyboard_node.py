#!/usr/bin/env python3
import getch
import rospy
from std_msgs.msg import String #String message 
from std_msgs.msg import Int8
import keyboard




def keys():
    rospy.init_node('keyboard_node',anonymous=True)
    pub = rospy.Publisher('/key',Int8,queue_size=10,latch=True )# "key" is the publisher name
    rospy.sleep(1)
    rate = rospy.Rate(100)#try removing this line ans see what happens
    while not rospy.is_shutdown():
        k=ord(getch.getch())# this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
        if ((k==119)|(k==115)|(k==100)|(k==97)|(k==113)):# to filter only the up , dowm ,left , right key /// this line can be removed or more key can be added to this
            rospy.loginfo(str(k))# to print on  terminal 
            rospy.sleep
            pub.publish(k)
        
        #rospy.loginfo(str(k))

        
            # while not rospy.is_shutdown():
            #     if keyboard.is_pressed('w'):
            #         pub.publish(119)
            #     elif keyboard.is_pressed('s'):
            #         pub.publish(115)
            #     elif keyboard.is_pressed('a'):
            #         pub.publish(97)
            #     elif keyboard.is_pressed('d'):
            #         pub.publish(100)
            #     elif keyboard.is_pressed('q'):
            #         pub.publish(113)
                    # k = input("Press a key (w, s, a, d, q): ")
                    # if k == 'w':
                    #     pub.publish(119)
                    # elif k == 's':
                    #     pub.publish(115)
                    # elif k == 'a':
                    #     pub.publish(97)
                    # elif k == 'd':
                    #     pub.publish(100)
                    # elif k == 'q':
                    #     pub.publish(113)


#s=115,e=101,g=103,b=98

if __name__=='__main__':
    try:
        keys()
        rospy.spin()

    except rospy.ROSInterruptException:
        #pass
        rospy.loginfo("Node")
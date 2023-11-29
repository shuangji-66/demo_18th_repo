#!/usr/bin/env python3
import rospy
from floor_set.msg  import Floor  
import sys, select, termios, tty
import time
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "speed is  %s", data.speed)
    
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("~/car/vel", Floor, callback)
    rospy.spin()

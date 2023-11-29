#!/usr/bin/env python3
import rospy
from floor_set.msg  import Floor  
import sys, select, termios, tty
import time

msg="""/dev/controler", 38400, timeout=0.05"""
import serial
import crcmod
import time
import struct

last_regnums = 0


def contro(speed,angle,headlight,horn,stop):

    ####################################################### 
    
    if int(speed) >= 1500 :
        back =0
    else :
        back =1
    state =    int( back)*4+  int(headlight)*4+ int(horn)*2+int(stop)

    sendbytes = add.to_bytes(1, byteorder="big", signed=False)+angle.to_bytes(2, byteorder="big", signed=False)+state.to_bytes(1, byteorder="big", signed=False)
    
    
    #######################################################    
  
    return sendbytes    
    
    

def callback(data):
    
    speed = data.speed
    angle = data.angle
    headlight = data.headlight
    horn = data.horn
    stop = data.stop
    
    rospy.loginfo(rospy.get_caller_id())
    rospy.loginfo( " speed is  %s ",speed)           
    rospy.loginfo( " angle is  %s ",angle)  
    rospy.loginfo( " headlight is  %s ",headlight)  
    rospy.loginfo( " horn is  %s ",horn)  
    rospy.loginfo( " stop is  %s ",stop)  
    #if last_regnums != regnums :
        
    send_data = contro(speed,angle,headlight,horn,stop)
    #com = serial.Serial("/dev/controler", 38400, timeout=0.05)
    #starttime = time.time()
    #com.write(send_data)
    rospy.loginfo( " send_data is  ")
    rospy.loginfo( send_data)
    #endtime = time.time()
    #print(f"used time: {endtime-starttime:.3f}")
    #com.close() 

        
        
if __name__ == '__main__':

    rospy.init_node('car_vel_controler')
    rospy.Subscriber("~/car_vel", Floor, callback)
    rospy.spin()

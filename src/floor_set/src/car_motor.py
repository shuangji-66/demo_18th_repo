#!/usr/bin/env python3
import rospy
from floor_set.msg  import Floor  
import sys, select, termios, tty
import time

msg="""dev/motor  9600 modbus-rtu   """
import serial
import crcmod
import time
import struct

last_regnums = 0
num = 0

def crc16(veritydata):
    if not veritydata:
        return
    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    return crc16(veritydata)

def mmodbus06(add, startregadd, regnum, funcode=6):
    if add < 0 or add > 0xFF or startregadd < 0 or startregadd > 0xFFFF or regnum < 0 or regnum > 0x801:
        print("Error: parameter error")
        return
    #if funcode != 6 :
    #    print("Error: parameter error")
    #    return
    sendbytes = add.to_bytes(1, byteorder="big", signed=False)
    sendbytes = sendbytes + funcode.to_bytes(1, byteorder="big", signed=False) + startregadd.to_bytes(2, byteorder="big", signed=False) + \
                regnum.to_bytes(2, byteorder="big", signed=False)
    crcres = crc16(sendbytes)
    crc16bytes = crcres.to_bytes(2, byteorder="little", signed=False)
    sendbytes = sendbytes + crc16bytes
    return sendbytes    
    
    

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "speed is  %s", data.speed)
    global last_regnums
    global num
    slaveadd = 1
    startreg = 0
    speed = data.speed

    if speed >=3000:
        regnums = 2048
    elif speed > 1500 and speed < 3001:
        regnums = 950 + int((speed-1500)/1500*1098)
    elif speed == 1500:
        regnums = 0X0       
    ###########################################################################
    elif speed < 1500 and speed >0 and num > 20:
        #regnums = int((1500 - speed)/1500*2048 )  
        regnums = 950 + int((1500-speed)/1500*1098)
        num +=1
    elif speed < 1500 and speed >0 and num < 20+1:
        regnums = 0X0   
    ###########################################################################   
    elif speed <= 0:
        regnums = 2048 
    rospy.loginfo(rospy.get_caller_id() + " regnums is  %s", regnums)           

    #################################
    if speed <1500  and num < 20+1:  #and last_regnums == regnums
        num += 1
    if speed > 1500 -1 and num > 20:
        num = 0
    #################################
    if last_regnums != regnums :

        send_data = mmodbus06(slaveadd, startreg, regnums)
        com = serial.Serial("/dev/motor", 9600, timeout=0.1)
        starttime = time.time()
        com.write(send_data)
        recv_data = com.read(regnums*2+5)
        endtime = time.time()
        if len(recv_data) > 0:
            print("recv: ", recv_data.hex())
        print(f"used time: {endtime-starttime:.3f}")
        com.close() 
        last_regnums = regnums  
        
        
if __name__ == '__main__':
    slaveadd = 1
    startreg = 0
    regnums = 0x800
    rospy.init_node('car_motor')
    rospy.Subscriber("~/car_vel", Floor, callback, queue_size=1)
    rospy.spin()

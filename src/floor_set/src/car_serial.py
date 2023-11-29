#!/usr/bin/env python3
msg="""dev/motor  9600 modbus-rtu   """
import serial
import crcmod
import time
import struct

# CRC16校验，返回整型数
def crc16(veritydata):
    if not veritydata:
        return
    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    return crc16(veritydata)

def mmodbus06(add, startregadd, regnum, funcode=6):
    if add < 0 or add > 0xFF or startregadd < 0 or startregadd > 0xFFFF or regnum < 1 or regnum > 0x801:
        print("Error: parameter error")
        return
    if funcode != 6 :
        print("Error: parameter error")
        return
    sendbytes = add.to_bytes(1, byteorder="big", signed=False)
    sendbytes = sendbytes + funcode.to_bytes(1, byteorder="big", signed=False) + startregadd.to_bytes(2, byteorder="big", signed=False) + \
                regnum.to_bytes(2, byteorder="big", signed=False)
    crcres = crc16(sendbytes)
    crc16bytes = crcres.to_bytes(2, byteorder="little", signed=False)
    sendbytes = sendbytes + crc16bytes
    return sendbytes


if __name__ == '__main__':
    slaveadd = 1
    startreg = 0
    regnums = 0x800
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



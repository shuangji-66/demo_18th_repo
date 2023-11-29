#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import serial
import struct
import rospy
import math
import platform
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf.transformations import quaternion_from_euler
import time
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]


if __name__ == "__main__":
    python_version = platform.python_version()[0]

    rospy.init_node("imu")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baud", 115200)
    print("IMU Type: Modbus Port:%s baud:%d" % (port, baudrate))
    imu_msg = Imu()
    mag_msg = MagneticField()
    gps_msg = NavSatFix()
    try:
        wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if wt_imu.isOpen():
            rospy.loginfo("\033[32mport open success...\033[0m")
        else:
            wt_imu.open()
            rospy.loginfo("\033[32mport open success...\033[0m")
    except Exception as e:
        print(e)
        rospy.loginfo("\033[31mport open failed\033[0m")
        exit(0)
    else:
        imu_pub = rospy.Publisher("wit/imu", Imu, queue_size=10)
        mag_pub = rospy.Publisher("wit/mag", MagneticField, queue_size=10)
        gps_pub = rospy.Publisher("wit/gps", NavSatFix, queue_size=10)

        master = modbus_rtu.RtuMaster(wt_imu)
        master.set_timeout(1)
        master.set_verbose(True)

        rate = rospy.Rate(10)  # hz
        while not rospy.is_shutdown():
            try:
                register = master.execute(80, cst.READ_HOLDING_REGISTERS, 52, 37)
            except Exception as e:
                print(e)
                rospy.loginfo(
                    "\033[31mread register time out, please check connection or baundrate set!\033[0m"
                )
                rate.sleep()
            else:
                reg_values = [0] * 37
                for i in range(0, 12):
                    if register[i] > 32767:
                        reg_values[i] = register[i] - 65536
                    else:
                        reg_values[i] = register[i]
                for i in range(12, 37):
                    reg_values[i] = register[i]

                acceleration = [reg_values[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                angularVelocity = [
                    reg_values[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3, 6)
                ]
                magnetometer = reg_values[6:9]
                angle_degree = [reg_values[i] / 32768.0 * 180 for i in range(9, 12)]

                heightL = reg_values[19]
                heightH = reg_values[20]
                longtitudeL = reg_values[21]
                longtitudeH = reg_values[22]
                latitudeL = reg_values[23]
                latitudeH = reg_values[24]
                altitude = reg_values[25]
                num_satellites = reg_values[33]
                gps_hdop = reg_values[35]

                height = (heightH * 65536 + heightL) / 100  # cm
                longtitude = (longtitudeH * 65536 + longtitudeL) / 10000000  # deg
                latitude = (latitudeH * 65536 + latitudeL) / 10000000  # deg
                altitude = altitude / 10  # m
                gps_hdop = gps_hdop / 100  # m

                stamp = rospy.get_rostime()

                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = "wit_imu_link"

                mag_msg.header.stamp = stamp
                mag_msg.header.frame_id = "wit_imu_link"

                gps_msg.header.stamp = stamp
                gps_msg.header.frame_id = "gps_antenna_link"
                gps_msg.status.service = NavSatStatus.SERVICE_GPS

                angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
                qua = quaternion_from_euler(
                    angle_radian[0], angle_radian[1], angle_radian[2]
                )

                imu_msg.orientation.x = qua[0]
                imu_msg.orientation.y = qua[1]
                imu_msg.orientation.z = qua[2]
                imu_msg.orientation.w = qua[3]

                imu_msg.angular_velocity.x = angularVelocity[0]
                imu_msg.angular_velocity.y = angularVelocity[1]
                imu_msg.angular_velocity.z = angularVelocity[2]

                imu_msg.linear_acceleration.x = acceleration[0]
                imu_msg.linear_acceleration.y = acceleration[1]
                imu_msg.linear_acceleration.z = acceleration[2]

                mag_msg.magnetic_field.x = magnetometer[0]
                mag_msg.magnetic_field.y = magnetometer[1]
                mag_msg.magnetic_field.z = magnetometer[2]

                gps_msg.latitude = latitude
                gps_msg.longitude = longtitude
                gps_msg.altitude = altitude
                gps_msg.status.status = NavSatStatus.STATUS_FIX
                # aproximate position_covariance from HDOP
                gps_msg.position_covariance[0] = gps_hdop**2
                gps_msg.position_covariance[4] = gps_hdop**2
                gps_msg.position_covariance[8] = (2 * gps_hdop) ** 2  # FIXME
                gps_msg.position_covariance_type = (
                    NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                )

                imu_pub.publish(imu_msg)
                mag_pub.publish(mag_msg)
                gps_pub.publish(gps_msg)

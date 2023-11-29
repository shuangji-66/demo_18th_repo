#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu


class ImuZeroing(object):
    def __init__(self) -> None:
        # ROS Infastructure
        rospy.init_node("imu_zeroing")
        self.sub_imu_raw = rospy.Subscriber(
            "imu_raw", Imu, self._recive_imu, queue_size=1
        )
        self.pub_imu = rospy.Publisher("imu", Imu, queue_size=1)

        self.isCalibrated = False
        self.offset = Imu()
        self.samples = []
        self.ignore_count = rospy.get_param("~ignore_count", 0)
        self.sample_count = rospy.get_param("~sample_count", 300)

    def average(self) -> Imu:
        # iterate samples and calculate average
        imu_avg = Imu()
        imu_avg.linear_acceleration.x = sum(
            [x.linear_acceleration.x for x in self.samples]
        ) / len(self.samples)
        imu_avg.linear_acceleration.y = sum(
            [x.linear_acceleration.y for x in self.samples]
        ) / len(self.samples)
        imu_avg.angular_velocity.z = sum(
            [x.angular_velocity.z for x in self.samples]
        ) / len(self.samples)
        rospy.loginfo("IMU calibration offset: %s", imu_avg)
        rospy.loginfo("IMU calibrated!!!")
        return imu_avg

    def _recive_imu(self, msg: Imu) -> None:
        if self.isCalibrated:
            result = Imu()
            result = msg
            result.linear_acceleration.x = (
                msg.linear_acceleration.x - self.offset.linear_acceleration.x
            )
            result.linear_acceleration.y = (
                msg.linear_acceleration.y - self.offset.linear_acceleration.y
            )
            result.angular_velocity.z = (
                msg.angular_velocity.z - self.offset.angular_velocity.z
            )

            self.pub_imu.publish(result)
        else:
            rospy.logwarn_once("Calibrating IMU, Stand Still!!")
            self.samples.append(msg)

    def run(self) -> None:
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if not self.isCalibrated:
                if len(self.samples) >= self.sample_count:
                    self.offset = self.average()
                    self.isCalibrated = True
            rate.sleep()


def main():
    node = ImuZeroing()
    node.run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit(0)

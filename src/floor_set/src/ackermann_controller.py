#!/usr/bin/python3

import rospy
from floor_set.msg import Floor
from ucar_common_msgs.msg import ExtraCtrl
from ackermann_msgs.msg import AckermannDrive
import math

class AckermannController(object):
    def __init__(self):
        self.speed_setpoint_ms = 0.0
        self.angle_setpoint_rad = 0.0
        self.lim_max_spd = 0.2
        self.lim_max_spd_backward = 0.1
        self.horn_setpoint = 0
        self.headlight_setpoint = 0
        self.break_setpoint = 0
        self.steer_offset_deg = -10

        # ROS Infastructure
        rospy.init_node("ackermann_controller")
        self.pub_ctrl_cmd = rospy.Publisher("/car_vel", Floor, queue_size=1)
        self.sub_ackermann = rospy.Subscriber(
            "/ackermann_cmd", AckermannDrive, self._recive_ackermann_cmd_callback, queue_size=1
        )
        self.sub_extra_ctrl = rospy.Subscriber(
            "/extra_ctrl", ExtraCtrl, self._recive_extra_contorl_callback, queue_size=1
        )

    def _recive_ackermann_cmd_callback(self, cmd):
        self.speed_setpoint_ms = cmd.speed
        self.angle_setpoint_rad = cmd.steering_angle

    def _recive_extra_contorl_callback(self, cmd):
        if cmd.horn:
            self.horn_setpoint = 1
        else:
            self.horn_setpoint = 0

        if cmd.headlight:
            self.headlight_setpoint = 1
        else:
            self.headlight_setpoint = 0

        if cmd.mechanical_break >= 0.99:
            self.break_setpoint = 1
        else:
            self.break_setpoint = 0

    def send_ctrl_cmd(self):
        msg = Floor()
        # msg.speed = 1500 + int(
        #     self.speed_setpoint_ms * 1500
        # )  
        msg.speed =int( 750*self.speed_setpoint_ms+1500)
                # TODO: map m/s to this 0-3000 nonsense
        # msg.speed = 1500
        # msg.angle = 90 + self.steer_offset_deg +int(
        #     self.angle_setpoint_rad * 90
        # )  
        msg.angle =int(-10.2673* -self.angle_setpoint_rad* -self.angle_setpoint_rad* -self.angle_setpoint_rad-6.3536* -self.angle_setpoint_rad* -self.angle_setpoint_rad-154.7130* -self.angle_setpoint_rad+91.4068) + self.steer_offset_deg
        # TODO: map rad to this 0-180 nonsense
        if msg.angle > 180:
            msg.angle = 180
        if msg.angle < 0:
            msg.angle = 0
        msg.headlight = int(self.headlight_setpoint)
        msg.horn = int(self.horn_setpoint)
        msg.stop = int(self.break_setpoint)
        self.pub_ctrl_cmd.publish(msg)

    def run(self):
        print("Ackermann Driver is running")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.send_ctrl_cmd()
            rate.sleep()

    def finalize(self):
        rospy.loginfo("Halting motors, aligning wheels and exiting...")
        halt_cmd_msg = Floor()
        halt_cmd_msg.speed = 1500
        halt_cmd_msg.angle = 90
        halt_cmd_msg.headlight = 0
        halt_cmd_msg.horn = 0
        halt_cmd_msg.stop = 0
        self.pub_ctrl_cmd.publish(halt_cmd_msg)


def main():
    node = AckermannController()
    rospy.on_shutdown(node.finalize)
    node.run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n")
        print("User Interrupt")
        exit(0)

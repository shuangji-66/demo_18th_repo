#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import tf
import actionlib
import time
import math
from enum import Enum
from copy import deepcopy
from std_srvs.srv import SetBool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID


# Define the navigation stage.
class NavStage(Enum):
    IDLE = 0
    TO_MID_POINT = 1
    ENROUTE_TO_MID_POINT = 2
    STOP = 3
    TO_HOME = 4
    ENROUTE_TO_HOME = 5


class UcarCommander(object):
    def __init__(self):
        # ROS Infastructure
        rospy.init_node("ucar_commander_node")
        self.srv_nav_start = rospy.Service(
            '/nav_start', SetBool, self.nav_start)
        # self.start_subscrber = rospy.Subscriber(
        #     "/nav_start", Bool, self.nav_start)
        # self.stop_publisher = rospy.Publisher(
        #     "/nav_stop", Bool, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.pub_movebase_goal = rospy.Publisher(
            "move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_move_base_abort = rospy.Publisher(
            "move_base/cancel", GoalID, queue_size=1)

        self.is_nav_start = False
        self.nav_stage = NavStage.IDLE
        self.mid_point_index = 0

        # get params from parameter server
        self.odom_frame_id = rospy.get_param(
            "~odom_frame_id", "odom")
        self.global_frame_id = rospy.get_param(
            "~global_frame_id", "map")
        self.robot_frame_id = rospy.get_param(
            "~robot_frame_id", "base_link")

        # goal params
        self.position = MoveBaseGoal()
        self.position.target_pose.header.frame_id = self.global_frame_id
        self.goal = MoveBaseGoal()
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.5)
        self.mid_point_num = rospy.get_param("~mid_point_num", 0)
        self.mid_point = []
        self.mid_point_tolerance = []
        mid_point = MoveBaseGoal()
        mid_point.target_pose.header.frame_id = self.global_frame_id
        for i in range(self.mid_point_num):
            self.mid_point_tolerance.append(deepcopy(
                rospy.get_param("~mid_point_tolerance", 10)[i]))
            mid_point.target_pose.pose.position.x = rospy.get_param(
                '~mid_point_position', [0.0, 0.0, 0.0])[i][0]
            mid_point.target_pose.pose.position.y = rospy.get_param(
                '~mid_point_position', [0.0, 0.0, 0.0])[i][1]
            mid_point.target_pose.pose.position.z = rospy.get_param(
                '~mid_point_position', [0.0, 0.0, 0.0])[i][2]
            mid_point.target_pose.pose.orientation.x = rospy.get_param(
                "~mid_point_orientation", [0.0, 0.0, 0.0, 0.0])[i][0]
            mid_point.target_pose.pose.orientation.y = rospy.get_param(
                "~mid_point_orientation", [0.0, 0.0, 0.0, 0.0])[i][1]
            mid_point.target_pose.pose.orientation.z = rospy.get_param(
                "~mid_point_orientation", [0.0, 0.0, 0.0, 0.0])[i][2]
            mid_point.target_pose.pose.orientation.w = rospy.get_param(
                "~mid_point_orientation", [0.0, 0.0, 0.0, 0.0])[i][3]
            self.mid_point.append(deepcopy(mid_point))

    # Link Start!!!
    def nav_start(self, req):
        if not self.is_nav_start:
            if req.data:
                # while not self.is_ready
                print("Action Stations!!!")
                self.nav_stage = NavStage.TO_MID_POINT
                self.is_nav_start = True
                self.goal_tolerance = deepcopy(
                    self.mid_point_tolerance[0])
                self.goal = deepcopy(self.mid_point[0])
                return True, "AccumviaTimer started, good luck."
            else:
                print("Ignoring request as the \"data\" field is set to false")
                return False, "False received"
        else:
            if req.data:
                print("Starting failed: already running")
                return False, "Already running"
            else:
                print("Abrot navigation is not implemented yet, doing nothing")
                return False, "Abrot navigation is not implemented yet, doing nothing"

    # linear distance
    def calculate_distance(self, point1, point2):
        distance = math.sqrt(
            (
                point1.target_pose.pose.position.x -
                point2.target_pose.pose.position.x
            ) ** 2 +
            (
                point1.target_pose.pose.position.y -
                point2.target_pose.pose.position.y
            ) ** 2
        )
        return distance

    # Just as its name.
    def _get_robot_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.global_frame_id, self.robot_frame_id, rospy.Time(0))
            self.position.target_pose.pose.position.x = trans[0]
            self.position.target_pose.pose.position.y = trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("tf lookup failed")

    # Send MoveBase goal coordinates.
    def send_movebase_goal(self, goal):
        self.move_base_client.send_goal(goal)
        simple_goal = PoseStamped()
        simple_goal.header.frame_id = self.global_frame_id
        simple_goal.pose.position.x = goal.target_pose.pose.position.x
        simple_goal.pose.position.y = goal.target_pose.pose.position.y
        simple_goal.pose.position.z = goal.target_pose.pose.position.z
        simple_goal.pose.orientation.x = goal.target_pose.pose.orientation.x
        simple_goal.pose.orientation.y = goal.target_pose.pose.orientation.y
        simple_goal.pose.orientation.z = goal.target_pose.pose.orientation.z
        simple_goal.pose.orientation.w = goal.target_pose.pose.orientation.w
        self.pub_movebase_goal.publish(simple_goal)

    # Disable the MoveBase framework.
    def abort_move_base(self):
        goal_id = GoalID()
        goal_id.stamp = rospy.Time.now()
        self.pub_move_base_abort.publish(goal_id)

    # The main process of the navigation.
    def flow(self):

        self._get_robot_position()

        if self.nav_stage == NavStage.IDLE:
            pass  # do nothing

        # cruise
        if self.mid_point_index < self.mid_point_num:
            if self.nav_stage == NavStage.TO_MID_POINT:
                print("Going to mid_point %d" %
                      ((self.mid_point_index + 1) % self.mid_point_num))
                self.goal = self.mid_point[(
                    self.mid_point_index) % self.mid_point_num]
                self.send_movebase_goal(self.goal)
                self.nav_stage = NavStage.ENROUTE_TO_MID_POINT

            if self.nav_stage == NavStage.ENROUTE_TO_MID_POINT:
                if self.calculate_distance(self.goal, self.position) <= \
                        self.mid_point_tolerance[(self.mid_point_index) % self.mid_point_num]:
                    self.mid_point_index += 1
                    if self.mid_point_index >= self.mid_point_num:
                        self.nav_stage = NavStage.STOP
                    else:
                        self.nav_stage = NavStage.TO_MID_POINT

        if self.nav_stage == NavStage.STOP:
            self.abort_move_base()
            self.nav_stage = NavStage.IDLE

    # You wonder what this thing is? Are you fucking teasing me?
    def run(self):
        rate = rospy.Rate(20)
        self.flow()
        while not rospy.is_shutdown():
            self.flow()
            rate.sleep()


# If you need this note, I will drag your head down and kick it as a ball.
def main():
    node = UcarCommander()
    node.run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)

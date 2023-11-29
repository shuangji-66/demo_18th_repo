/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2020:
     - chentairan <tairanchen@bitfsd.cn>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "planning_node_handle.hpp"

namespace ns_planning_node
{

  // Constructor
  plannerHandle::plannerHandle(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle),
                                                              planning_node_(nodeHandle)
  {
    ROS_INFO("Constructing Handle");
    loadParameters();
    subscribeToTopics();
    publishToTopics();
    run();
  }

  // Getters
  int plannerHandle::getNodeRate() const { return node_rate_; }

  // Methods
  void plannerHandle::loadParameters()
  {
    ROS_INFO("loading handle parameters");
    if (!nodeHandle_.param<std::string>("input_cone_topic_name",
                                        input_cone_topic_name_,
                                        "/map"))
    {
      ROS_WARN_STREAM("Did not load input_cone_topic_name. Standard value is: " << input_cone_topic_name_);
    }
    if (!nodeHandle_.param<std::string>("output_trajectory_topic_name",
                                        output_trajectory_topic_name_,
                                        "/trajectory"))
    {
      ROS_WARN_STREAM("Did not load output_trajectory_topic_name. Standard value is: " << output_trajectory_topic_name_);
    }
    if (!nodeHandle_.param("node_rate", node_rate_, 1))
    {
      ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
    }
  }

  void plannerHandle::subscribeToTopics()
  {
    ROS_INFO("subscribe to topics");
    inputConeSubscriber_ = nodeHandle_.subscribe(input_cone_topic_name_, 1, &plannerHandle::inputConeCallback, this);
  }

  void plannerHandle::publishToTopics()
  {
    ROS_INFO("publish to topics");
    ouputTrajectoryPublisher_ = nodeHandle_.advertise<ucar_common_msgs::Trajectory>(output_Trajectory_topic_name_, 1);
  }
  void plannerHandle::inputConeCallback(const ucar_common_msgs::map &msg)
  {
    planner_.getCone(msgs);
  }
  void plannerHandle::run()
  {
    planning_node_.runAlgorithm();
    sendMsg();
  }

  void plannerHandle::sendMsg()
  {
    ouputPosePublisher_.publish(planning_node_.sendtrajectory());
  }

  void plannerHandle::inputPoseCallback(const ucar_common_msgs::map &msg)
  {
    planning_node_.setCone(msg);
  }
}
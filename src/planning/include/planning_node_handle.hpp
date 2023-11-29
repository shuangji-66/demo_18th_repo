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

#ifndef PLANNING_NODE_HANDLE_HPP
#define PLANNING_NODE_HANDLE_HPP

#include "planning_node.hpp"

namespace ns_planning_node
{

  class plannerHandle
  {

  public:
    // Constructor
    plannerHandle(ros::NodeHandle &nodeHandle);

    //  // Getters
    int getNodeRate() const;

    // Methods
    void loadParameters();
    void subscribeToTopics();
    void publishToTopics();
    void run();
    void sendMsg();
    //  void sendVisualization();

  private:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber inputConeSubscriber_;
    ros::Publisher ouputTrajectoryPublisher_;

    void inputConeCallback(const ucar_common_msgs::map &msg);

    std::string input_cone_topic_name_;
    std::string output_trajectory_topic_name_;
    planner planner_;

    int node_rate_;

    planner planning_node_;
  };
}

#endif // PLANNING_NODE_HANDLE_HPP

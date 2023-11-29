/*

    Copyright (c) 2020:
     - chentairan <tairanchen@bitfsd.cn>

    Smart-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Smart-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Smart-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef CONTROLLER_HANDLE_HPP
#define CONTROLLER_HANDLE_HPP

#include "controller.hpp"

namespace ns_controller
{

  class controlHandle
  {

  public:
    // Constructor
    controlHandle(ros::NodeHandle &nodeHandle);

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
    ros::Subscriber inputimuSubscriber_;
    ros::Subscriber inputlocationSubscriber_;
    ros::Subscriber inputtrajectorySubscriber_;
    ros::Publisher ouputPosePublisher_;

    void GetImuMessagecallback(const sensor_msgs::Imu::ConstPtr &msgs);
    void GetLocationMessagecallback(const sensor_msgs::NavSatFix::ConstPtr &msgs);
    void GetTrajectoryMessagecallback(const ucar_common_msgs::Trajectory &msgs);

    std::string input_imu_topic_name_;
    std::string input_location_topic_name_;
    std::string input_trajectory_topic_name_;
    std::string output_pose_topic_name_;

    int node_rate_;

    control controller_;
  };
}

#endif // CONTROLLER_HANDLE_HPP

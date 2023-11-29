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

#ifndef PLANNING_NODE_HPP
#define PLANNING_NODE_HPP

#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "ucar_common_msgs/map.h"
#include "ucar_common_msgs/cone.h"
#include "ucar_common_msgs/Trajectory.h"
#include "ucar_common_msgs/TrajectoryPoint.h"
#include "type.h"
#include "planner_base.h"
#include "planner1.h"
namespace ns_planning_node
{

  class planner
  {
  public:
    // Constructor
    planner(ros::NodeHandle &nh);
    // Getters
    void getCone(ucar_common_msgs::map msg);
    void runAlgorithm();
    Trajectory sendtrajectory();

  private:
    ros::NodeHandle &nh_;

    Trajectory trajectory_output;
    RedCone redcone;
    BlueCone bluecone;
    Planner *planning_;
    Planner1 planner1_;
  };
}

#endif // PLANNING_NODE_HPP

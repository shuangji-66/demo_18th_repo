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
#include "planning_node.hpp"
#include <sstream>

namespace ns_planning_node
{
    // Constructor
    planner::planner(ros::NodeHandle &nh) : nh_(nh)
    {
        planning_ = &planner1_;
    };

    // Getters
    void planner::getCone(ucar_common_msgs::map msg)
    {
        redcone = msg.cone_red;
        bluecone = msg.cone_blue;
    }

    Trajectory planner::sendtrajectory()
    {
        return trajectory_output;
    }
    void planner::runAlgorithm()
    {
        planning_->ConeClassify();
        planning_->solve();
        this->trajectory_output=planning_->getresult();

    }

}

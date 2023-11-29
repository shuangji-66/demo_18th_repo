/*
    SmartCar Project(Smart-Project).
    Copyright (c) 2023:
     - shuangji<xushuangzhe@qq.com>

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

#include <ros/ros.h>
#include "controller_handle.hpp"

typedef ns_controller::controlHandle controlHandle;

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller");
  ros::NodeHandle nodeHandle("~");
  controlHandle mycontrolHandle(nodeHandle);
  ros::Rate loop_rate(mycontrolHandle.getNodeRate());
  while (ros::ok()) {

    mycontrolHandle.run();

    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}


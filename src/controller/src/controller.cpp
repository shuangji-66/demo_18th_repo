/*
    Formula Student Driverless Project (Smart-Project).
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

#include <ros/ros.h>
#include "controller.hpp"
#include <sstream>

namespace ns_controller
{
    // Constructor
    control::control(ros::NodeHandle &nh) : nh_(nh)
    {
        solver_ = &stanley_solver_;
    };

    // Getters
    // floor_msg::Floor control::sendcmd()
    // {
    //     floor_msg::Floor output;
    //     output.angle = this->result[0];
    //     output.speed = this->result[1];
    //     output.headlight = 1;
    //     output.horn = 0;
    //     output.stop = this->result[2];
    //     return output;
    // }
    ackermann_msgs::AckermannDrive control::sendcmd()
    {
        ackermann_msgs::AckermannDrive output;
        output.steering_angle = result[0];
        output.speed = result[1];
        std::cout << "output.speed" << result[1] << std::endl;
        output.acceleration = 0.0;
        output.jerk = 0.0;
        output.steering_angle_velocity = 0.0;
        return output;
    }

    // Setters
    void control::setimu(const sensor_msgs::Imu::ConstPtr &msg)
    {
        geometry_msgs::Quaternion quat = msg->orientation;
        tf::Quaternion tf_quat;
        tf::quaternionMsgToTF(quat, tf_quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        carstate.yaw = yaw;
        carstate.r = msg->angular_velocity.z;
        carstate.ax = msg->linear_acceleration.x;
        carstate.ay = msg->linear_acceleration.y;
    }
    void control::setlocation(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        // 地球半径（单位：千米）
        const double EARTH_RADIUS = 6371.0;

        // 参考经度和纬度 北京
        const double REFERENCE_LONGITUDE = 39.9042;
        const double REFERENCE_LATITUDE = 116.4074;

        // 从消息中获取GPS经纬度信息
        double latitude = msg->latitude;
        double longitude = msg->longitude;

        // 将经纬度坐标转换为弧度
        double latitude_rad = latitude * M_PI / 180.0;
        double longitude_rad = longitude * M_PI / 180.0;
        double reference_longitude_rad = REFERENCE_LONGITUDE * M_PI / 180.0;
        double reference_latitude_rad = REFERENCE_LATITUDE * M_PI / 180.0;

        // 计算全局x和y坐标
        double x = EARTH_RADIUS * cos(latitude_rad) * cos(longitude_rad - reference_longitude_rad);
        double y = EARTH_RADIUS * cos(latitude_rad) * sin(longitude_rad - reference_longitude_rad);

        // 现在你可以使用x和y坐标进行后续操作
        // carstate.x = x;
        // carstate.y = y;
    }
    void control::settrajectory(const ucar_common_msgs::Trajectory &msg)
    {
        refline_ = msg;
    }
    void control::runAlgorithm()
    {
        solver_->setTrajectory(refine_);
        std::cout << "runaaaaaaa" << std::endl;
        solver_->setState(carstate);
        solver_->solve();
        this->result = solver_->getresult();
    }

}

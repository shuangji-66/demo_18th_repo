#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

namespace ns_controller
{
    struct VehicleState
    {

        double yaw;
        double r;
        double ax;
        double ay;
        VehicleState()
        {
            x = 0;
            y = 0;
            yaw = 0;
            v = 0;
            r = 0;
            a = 0;
            vx = 0;
            vy = 0;
            ax = 0;
            ay = 0;
        }
    };
    struct TrajectoryPoint
    {
        double x;
        double y;
        double yaw;
        double curvature;
        double velocity;
    };
    typedef std::vector<TrajectoryPoint> Trajectory;
}
#ifndef TYPE_H_
#define TYPE_H_

#include "ucar_common_msgs/map.h"
#include <cmath>
#include <vector>
#include <map>

namespace ns_planning_node
{
    typedef struct
    {
        double x;
        double y;
        double yaw;
        double curvature;
        double velocity;
        double yaw_rate;
        double acc;

    } TrajectoryPoint;
    typedef std::vector<TrajectoryPoint> Trajectory;
    typedef struct
    {
        float x; // 相对车的坐标
        float y;
        int color; // red==0;blue==1;yellow==2;unknow==3

    } ConeDef;
    typedef std::vector<ConeDef>RedCone;
    typedef std::vector<ConeDef>BlueCone;

}
#endif
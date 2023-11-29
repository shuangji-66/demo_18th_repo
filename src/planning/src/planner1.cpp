#include "ros/ros.h"
#include <vector>
#include <algorithm>
#include "planning_node.hpp"
#include "planner1.h"
#include <algorithm>
#include "fsd_tools/cubic_spline.h"
namespace ns_planning_node
{
    void Planner1::solve()
    {
        if (!gloabl_planner())
            std::cout << "gloabl_planner failed" << std::endl;
    }
    bool Planner1::gloabl_planner()
    {
        if (this->redcone_.empty() || this->bluecone_.empty())
            return 0;
        std::vector<double> wx, wy;
        wx.emplace_back(0);
        wy.emplace_back(0);
        for (const auto &red : redcone_)
        {
            const auto BlueConeBegin = bluecone_.begin();
            const auto BlueConeEnd = bluecone_.end();
            const auto it_blue = std::min_element(BlueConeBegin, BlueConeEnd, [&](const ConeDef &a, const ConeDef &b)
                                                  {
                const double da=std::hypot(red.x-a.x,red.y-a.y);
                const double db=std::hypot(red.x-b.y,red.y-b.y);
                return da<db; });
            float x = static_cast<float>((red.x + it_blue->x) / 2.0);
            float y = static_cast<float>((red.y + it_blue->y) / 2.0);
            wx.emplace_back(x);
            wy.emplace_back(y);
        }
        fsd::Spline2D spline(wx, wy);
        double interval = 0.5; // 标定量，不知道是啥
        TrajectoryPoint temp;
        for (float i = 0; i < spline.s.back(); i += interval)
        {
            std::array<float, 2> point_ = spline.calc_position(i);
            temp.x = point_[0];
            temp.y = point_[1];
            temp.r = 0;
            temp.curvature = spline.calc_curvature(i);
            temp.yaw = spline.calc_yaw(i);
            trajectory_.emplace_back(temp);
        }
        float dis_min = 1;
        bool flag1 = 0;
        bool flag2 = 0;
        float index = 0;
        int traj_back = trajectory_.size() - 1;
        for (int i = traj_back; i >= 0; --i)
        {
            double dis = std::hypot(trajectory_[traj_back].x - trajectory_[i].x,
                                    trajectory_[traj_back].y - trajectory_[i].y);
            if (flag1 == false && dis <= dis_min)
                continue;
            if (flag1 == false && dis > dis_min)
                flag1 = true;
            if (flag1 == true && dis <= dis_min)
                flag2 = true;
            if (flag2 == true && dis <= dis_min)
                dis_min = dis;
            if (flag2 == true && dis > dis_min)
            {
                index = i;
                break;
            }
        }
        auto tmp = new Trajectory;
        for (int i = index; i < trajectory_.size(); ++i)
        {
            tmp->push_back(trajectory_[i]);
        }
        trajectory_.clear();
        trajectory_ = *tmp;
        delete tmp;
    }
    bool Planner1::trajectory_planner()
    {
    }
}
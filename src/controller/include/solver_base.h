#pragma once

#include "types.h"
#include "ackermann_msgs/AckermannDrive.h"
namespace ns_controller
{
    class Solver
    {
    public:
        void setTrajectory(const Trajectory &trajectory);
        void setState(VehicleState state);

        Trajectory predictive_path;
        std::vector<double> getresult();
        virtual void solve() = 0;

    protected:
        Trajectory trajectory_;
        VehicleState state_;
       
        ackermann_msgs::AckermannDrive control_commend_;
    };
}
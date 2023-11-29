#include "solver_base.h"

namespace ns_controller
{
    void Solver::setTrajectory(const Trajectory &trajectory)
    {
        this->trajectory_ = trajectory;
    }
    void Solver::setState(VehicleState state)
    {

        this->state_ = state;
      }

    std::vector<double> Solver::getresult()
    {
        std::vector<double> output;
        output.emplace_back(control_commend_.steering_angle);

        output.emplace_back(control_commend_.speed);

        return output;
    }

}
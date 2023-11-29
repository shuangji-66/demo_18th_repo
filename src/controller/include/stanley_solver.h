# ifndef STANLEY_SOLVER_H_
# define STANLEY_SOLVER_H_
#include "solver_base.h"
#include "pid_controller.h"

namespace ns_controller
{
    class Stanley : public Solver
    {
    public:
        void solve();
        int findnearestindex(const Trajectory &traj, const VehicleState &state);
        void ComputeControlCmd(const Trajectory &traj, const VehicleState &state);
        void ComputeLateralErrors(const Trajectory &traj, const double x, const double y, const double theta,
                                  double &e_y, double &e_theta);
        double PointDistanceSquare(const double &x,const double &y,const VehicleState &state);
    };

}
#endif

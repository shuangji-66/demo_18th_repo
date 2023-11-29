#ifndef PLANNER1_H_
#define PLANNER1_H_
#include "planner_base.h"

namespace ns_planning_node
{
    class Planner1 : public Planner
    {
    public:
        void solve();
        bool gloabl_planner();
        bool trajectory_planner();
    };

}

#endif
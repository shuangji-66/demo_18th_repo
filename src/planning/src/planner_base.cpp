#include "planner_base.h"

namespace ns_planning_node
{
    void Planner::ConeClassify()
    {
        redcone_=redcone;
        bluecone_=bluecone;
    }
    Trajectory Planner::getresult()
    {
        return trajectory_;
    }
}
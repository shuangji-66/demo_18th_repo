#ifndef PLANNER_BASE_H_
#define PLANNER_BASE_H_
#include "type.h"
#include "planning_node.hpp"
namespace ns_planning_node
{
    class Planner
    {
    public:
        void ConeClassify();
        Trajectory getresult();
        virtual void solve() = 0;

    protected:
        Trajectory trajectory_;
        Conearray cone;
        // redcone需要先定义好数组size
        RedCone redcone_;
        BlueCone bluecone_;
    };
}
#endif
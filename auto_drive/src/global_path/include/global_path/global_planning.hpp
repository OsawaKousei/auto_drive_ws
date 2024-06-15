#ifndef GLOBAL_PLANNING_HPP
#define GLOBAL_PLANNING_HPP

#include <utility>
#include <vector>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Cost.h>
#include "ompl/util/Console.h"
#include <boost/program_options.hpp>
#include <memory>
#include <fstream>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <global_path/planner_defines.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using pose = geometry_msgs::msg::Pose;

namespace global_path{

class OMPL_PlannerClass{
private:
    std::shared_ptr<ompl::base::SpaceInformation> _space_info_base_area;  // ベースのフィールド
public:
    OMPL_PlannerClass();
    std::vector<pose> plan(const pose& start_state, const pose& goal_state);
};

} // namespace global_path
#endif //GLOBAL_PLANNING_HPP
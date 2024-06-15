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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace global_path{

enum PlannerType{  // Ubuntu22.04ならもっと多くの種類が使えるはず
    PLANNER_BFMTSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
};

class BaseArea : public ob::StateValidityChecker{  // state spaceのvalidity checker
    ob::SpaceInformationPtr space_info;
    nav_msgs::msg::OccupancyGrid space_shapes;
    double _robot_r = 10.0;

public:
    explicit BaseArea(const ob::SpaceInformationPtr& space_info_, const nav_msgs::msg::OccupancyGrid &space_shapes_);
    bool isValid(const ob::State* state) const override;
};

class BaseAreaMotionValidator : public ob::MotionValidator{ // motion validator
    ob::SpaceInformationPtr space_info;
    nav_msgs::msg::OccupancyGrid space_shapes;
    double _robot_r = 10.0;
public:
    explicit BaseAreaMotionValidator(const ob::SpaceInformationPtr& space_info_, const nav_msgs::msg::OccupancyGrid &space_shapes_);
    bool checkMotion(const ob::State* s1, const ob::State* s2) const override;
    bool checkMotion(const ob::State* s1, const ob::State* s2, std::pair<ob::State*, double>& lastValid) const override;
};

class OMPL_PlannerClass{
private:
    std::shared_ptr<ompl::base::SpaceInformation> _space_info_base_area;  // ベースのフィールド
public:
    OMPL_PlannerClass(const nav_msgs::msg::OccupancyGrid &map, double robot_size);
    nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose &start_state, const geometry_msgs::msg::Pose &goal_state);
};

} // namespace global_path
#endif //GLOBAL_PLANNING_HPP
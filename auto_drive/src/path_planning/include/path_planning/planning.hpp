//
// Created by emile on 24/05/08.
//

#ifndef PLANNER_CPP_PLANNING_HPP
#define PLANNER_CPP_PLANNING_HPP

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
#include "planner_defines.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;



class BaseArea : public ob::StateValidityChecker{  // state spaceのvalidity checker
    ob::SpaceInformationPtr space_info;
    double _robot_r = 10.0;

public:
    std::vector<std::shared_ptr<SpaceShapes>> space_shapes;  // 障害物の形状
    explicit BaseArea(const ob::SpaceInformationPtr& space_info_, const std::vector<std::shared_ptr<SpaceShapes>>& space_shapes_);
    double clearance(const ob::State* state) const override;
    // 与えられた状態の位置が円形の障害物に重なっているかどうかを返す。

    bool isValid(const ob::State* state) const override;
};

class BaseAreaMotionValidator : public ob::MotionValidator{
    ob::SpaceInformationPtr space_info;
    double _robot_r = 10.0;
public:
    std::vector<std::shared_ptr<SpaceShapes>> space_shapes;  // 障害物の形状
    explicit BaseAreaMotionValidator(const ob::SpaceInformationPtr& space_info_, const std::vector<std::shared_ptr<SpaceShapes>>& space_shapes_);
    bool checkMotion(const ob::State* s1, const ob::State* s2) const override;
    bool checkMotion(const ob::State* s1, const ob::State* s2, std::pair<ob::State*, double>& lastValid) const override;
};


class PathLengthObjective : public ob::StateCostIntegralObjective{
public:
    PathLengthObjective(const ob::SpaceInformationPtr& space_info);

    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override;
};


class ClearanceObjective : public ob::StateCostIntegralObjective{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& space_info);

    ob::Cost stateCost(const ob::State* s) const override;
};


class OMPL_PlannerClass{
private:
    std::shared_ptr<ompl::base::SpaceInformation> _space_info_base_area;  // ベースのフィールド
public:
    OMPL_PlannerClass();
    std::vector<RobotState> plan(const RobotState& start_state, const RobotState& goal_state);
    // is_feasible (trueなら問題ない)
};


#endif //CPPCODES_PLANNING_HPP

//
// Created by emile on 24/05/08.
//


#include <planning.hpp>
#include <util_functions.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

// For boost program options
#include <boost/program_options.hpp>
#include <memory>
#include <fstream>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/goals/GoalState.h>


#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <utility>
#include <fstream>
#include <limits>


template<class T> using matrix= std::vector<std::vector<T>>;


ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr space_info, PlannerType plannerType){
    switch (plannerType){
        case PLANNER_BFMTSTAR:
            return std::make_shared<og::BFMT>(space_info);
            break;
        case PLANNER_CFOREST:
            return std::make_shared<og::CForest>(space_info);
            break;

        case PLANNER_FMTSTAR:
            return std::make_shared<og::FMT>(space_info);
            break;

        case PLANNER_INF_RRTSTAR:
            return std::make_shared<og::InformedRRTstar>(space_info);
            break;
        case PLANNER_PRMSTAR:
            return std::make_shared<og::PRMstar>(space_info);
            break;
        case PLANNER_RRTSTAR:
            return std::make_shared<og::RRTstar>(space_info);
            break;
        case PLANNER_SORRTSTAR:
            return std::make_shared<og::SORRTstar>(space_info);
            break;
        default:
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
    }
}



std::vector<XY> rectangle_vertexes(double w, double h, double x, double y, double theta){
    std::vector<XY> tmp = {{w/2.0, h/2.0}, {-w/2.0, h/2.0}, {-w/2.0, -h/2.0}, {w/2.0, -h/2.0}};
//    std::for_each(tmp.begin(), tmp.end(), [theta, x, y](auto& point){point = rot2D(point, theta)+XY(x, y);});
    for(auto& v: tmp){
        v = rot2D(v, theta) + XY({x, y});
    }
    return tmp;
}


BaseArea::BaseArea(const ompl::base::SpaceInformationPtr &space_info_, const std::vector<std::shared_ptr<SpaceShapes>>& space_shapes_)
: StateValidityChecker(space_info_) {
    space_info = space_info_;
    std::copy(space_shapes_.begin(), space_shapes_.end(), std::back_inserter(space_shapes));
}

double BaseArea::clearance(const ob::State *state) const { // 与えられた状態の位置から障害物までの最短距離を返す。
    XY point = get2DPos(state);

    std::vector<double> clearances;
    std::transform(space_shapes.begin(), space_shapes.end(), std::back_inserter(clearances),
                   [point](auto& shape){return shape->clearance(point, 0.0);});
    return *std::min_element(clearances.begin(), clearances.end());
}

bool BaseArea::isValid(const ob::State *state) const {
    return this->clearance(state) > 0.0 && this->space_info->satisfiesBounds(state);
}


BaseAreaMotionValidator::BaseAreaMotionValidator(const ompl::base::SpaceInformationPtr &space_info_,
                                                 const std::vector<std::shared_ptr<SpaceShapes>>& space_shapes_)
: MotionValidator(space_info_){
    space_info = space_info_;
    std::copy(space_shapes_.begin(), space_shapes_.end(), std::back_inserter(space_shapes));
}

bool BaseAreaMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const {
    XY s1_xy = get2DPos(s1);
    XY s2_xy = get2DPos(s2);

    bool is_valid = std::all_of(space_shapes.begin(), space_shapes.end(),
                                [&s1_xy, &s2_xy](const auto& shape){return shape->check_valid(s1_xy, s2_xy);});
    return is_valid;
}

bool BaseAreaMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2,
                                          std::pair<ob::State *, double> &lastValid) const {
    if(this->checkMotion(s1, s2))return true;  // trueを返す場合はlastValidは変更しない

    lastValid.second = 0.0;
    // TODO: 実装 (lastValidを更新する. secondは0~1の値で, 0の場合はs1, 1の場合はs2)
    return false;
}


PathLengthObjective::PathLengthObjective(const ompl::base::SpaceInformationPtr &space_info):
    ob::StateCostIntegralObjective(space_info, true){
}

ob::Cost PathLengthObjective::motionCost(const ob::State *s1, const ob::State *s2) const {
    return StateCostIntegralObjective::motionCost(s1, s2);
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si){
    auto opt = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    std::function<ob::Cost(const ob::State*, const ob::Goal*)> h_func = [](const ob::State *s, const ob::Goal *g){
        auto goal = g->getSpaceInformation()->allocState();
        auto[x1, y1] = get2DPos(s);
        auto[x2, y2] = get2DPos(goal);
        return ob::Cost((pow(x2-x1, 2.0) + pow(y2-y1, 2.0)));
    };

    opt->setCostToGoHeuristic(h_func);
    return opt;
}



ClearanceObjective::ClearanceObjective(const ob::SpaceInformationPtr& space_info)
        :ob::StateCostIntegralObjective(space_info, true){
}

ob::Cost ClearanceObjective::stateCost(const ob::State* s) const {
    // 障害物との距離を最大化する
    return ob::Cost(log(1 / (si_->getStateValidityChecker()->clearance(s) + std::numeric_limits<double>::min())));
}



OMPL_PlannerClass::OMPL_PlannerClass() {
    auto state_space_base_area = std::make_shared<ob::RealVectorStateSpace>(2);
    matrix<double> bounds_pre_base_area{  // 元のstateの範囲
            {-100, 100},
            {-30, 30}
    };
    ob::RealVectorBounds bounds_base_area(2);
    for(int i=0; i<bounds_pre_base_area.size(); i++){
        bounds_base_area.setLow(i, bounds_pre_base_area[i][0]);
        bounds_base_area.setHigh(i, bounds_pre_base_area[i][1]);
    }
    state_space_base_area->setBounds(bounds_base_area);  // bounds for param

    std::vector<std::shared_ptr<SpaceShapes>> space_shapes = {  // fieldの定義
            std::shared_ptr<SpaceShapes>(new RectangleSpace({-100, -30}, {100, 30}, false)),
            std::shared_ptr<SpaceShapes>(new RectangleSpace({2, 2}, {4, 4}, true)),
            std::shared_ptr<SpaceShapes>(new CircleSpace({7, 7}, 2, true)),
    };  // space shapes

    _space_info_base_area = std::make_shared<ob::SpaceInformation>(state_space_base_area);
    _space_info_base_area->setStateValidityChecker(std::make_shared<BaseArea>(_space_info_base_area, space_shapes));  // checker
    _space_info_base_area->setMotionValidator(std::make_shared<BaseAreaMotionValidator>(_space_info_base_area, space_shapes));  // motion validator
    _space_info_base_area->setup();
}


std::vector<RobotState> OMPL_PlannerClass::plan(const RobotState &start_state, const RobotState &goal_state) {
    auto state_space(std::make_shared<ob::RealVectorStateSpace>(2));
    auto space_info = _space_info_base_area;

    // start point
    ob::ScopedState<> start(state_space);
    std::cout << "start: x,y,theta -> " << start_state.x << ", " << start_state.y << ", " << start_state.theta << std::endl;
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_state.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_state.y;


    // goal point
    ob::ScopedState<> goal(state_space);
    std::cout << "goal: x,y,theta -> " << goal_state.x << ", " << goal_state.y << ", " << goal_state.theta << std::endl;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_state.x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_state.y;

    // problem definition
    auto prob_def(std::make_shared<ob::ProblemDefinition>(space_info));  // problem instance
    prob_def->setStartAndGoalStates(start, goal);
    prob_def->setOptimizationObjective(getPathLengthObjective(space_info));  // 目的関数の設定
    //    prob_def->print(std::cout);  // 問題設定を表示

    auto planner = allocatePlanner(space_info, PLANNER_INF_RRTSTAR);

    // planner settings
    double start_goal_dist = length({start_state.x, start_state.y}, {goal_state.x, goal_state.y});
    planner.get()->as<og::InformedRRTstar>()->setGoalBias(0.1);
    planner.get()->as<og::InformedRRTstar>()->setRange(start_goal_dist/4.0*3.0);

    planner->setProblemDefinition(prob_def);  // problem instanceを代入
    planner->setup();  // plannerのsetup
    planner->checkValidity();

    ob::PlannerStatus solved = planner->ob::Planner::solve(0.002);
    if (!solved) {
        std::cout << "No solution found" << std::endl;
        return std::vector<RobotState>{};
    }

    auto path = std::static_pointer_cast<og::PathGeometric>(prob_def->getSolutionPath());
    if(path->getStates().size() <= 2){  // スプライン補間は3点以上必要
        path->interpolate(3);
    }
    path->checkAndRepair(5);

    std::vector<RobotState> traj;
    for(auto& tmp: path->getStates()){
        auto [x, y] = get2DPos(tmp);
        traj.emplace_back(x, y, 0.0);
    }
    return traj;

//    std::vector<RobotState> traj_pre = path_func(traj, 0.3);  // スプライン補間 (パラメータ空間)
//    std::for_each(traj_pre.begin(), traj_pre.end(), [](auto& tmp){ tmp = clip_arm_state(tmp);});
//    std::vector<RobotState> r_theta_trajectory = path_func_xy(traj_pre, l_min, l_max, d_max);  // スプライン補間 (xy)
//    return std::make_pair(r_theta_trajectory, true);
}


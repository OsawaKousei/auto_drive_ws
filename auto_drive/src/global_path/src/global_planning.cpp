#include "global_path/global_planning.hpp"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

template<class T> using matrix= std::vector<std::vector<T>>;

namespace global_path{
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

  std::tuple<double, double> get2DPos(const ompl::base::State *state) {
    const auto *r_vec = state->as<ob::RealVectorStateSpace::StateType>();
    const double *tmp = r_vec->values;
    auto [a, b] = std::make_tuple(tmp[0], tmp[1]);
    return std::make_tuple(a, b);
  }

  BaseArea::BaseArea(const ob::SpaceInformationPtr& space_info_, const nav_msgs::msg::OccupancyGrid &space_shapes_): ob::StateValidityChecker(space_info_){
    space_info = space_info_;
    space_shapes = space_shapes_;
  }

  bool BaseArea::isValid(const ob::State* state) const{
    auto [x, y] = get2DPos(state);
    double map_resolution = space_shapes.info.resolution;

    if(!space_info->satisfiesBounds(state)){
      std::cout << "out of map" << std::endl;
      return false;
    }
    // stateが重なるgridを取得
    int x_idx = int(x/map_resolution);
    int y_idx = int(y/map_resolution);
    // gridが障害物の場合はfalseを返す
    if(space_shapes.data[y_idx*space_shapes.info.width + x_idx] > 0){
      return false;
    }
    return true;
  }

  BaseAreaMotionValidator::BaseAreaMotionValidator(const ob::SpaceInformationPtr& space_info_, const nav_msgs::msg::OccupancyGrid &space_shapes_): ob::MotionValidator(space_info_){
    space_info = space_info_;
    space_shapes = space_shapes_;
  }

  bool BaseAreaMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const{
    auto [x1, y1] = get2DPos(s1);
    auto [x2, y2] = get2DPos(s2);
    double map_resolution = space_shapes.info.resolution;
    if(!space_info->satisfiesBounds(s1) || !space_info->satisfiesBounds(s2)){
      return false;
    }
    // 2点間の直線を求める
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
    double theta = atan2(dy, dx);
    // 2点間の直線上の点を取得
    for(double i=0; i<dist; i+=map_resolution){
      double x = x1 + i*cos(theta);
      double y = y1 + i*sin(theta);
      // gridが障害物の場合はfalseを返す
      int x_idx = int(x/map_resolution);
      int y_idx = int(y/map_resolution);
      if(space_shapes.data[y_idx*space_shapes.info.width + x_idx] > 0){
        return false;
      }
    }
    return true;
  }

  bool BaseAreaMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2, std::pair<ob::State*, double>& lastValid) const{
    if(this->checkMotion(s1, s2))return true;  // trueを返す場合はlastValidは変更しない

    lastValid.second = 0.0;
    // TODO: 実装 (lastValidを更新する. secondは0~1の値で, 0の場合はs1, 1の場合はs2)
    return false;
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

  OMPL_PlannerClass::OMPL_PlannerClass(const nav_msgs::msg::OccupancyGrid &map){
    auto state_space_base_area = std::make_shared<ob::RealVectorStateSpace>(2);
    matrix<double> bounds_pre_base_area{  // 元のstateの範囲
            {0, double(map.info.width)},
            {0, double(map.info.height)}
    };
    ob::RealVectorBounds bounds_base_area(2);
    for(int i=0; i<bounds_pre_base_area.size(); i++){
        bounds_base_area.setLow(i, bounds_pre_base_area[i][0]);
        bounds_base_area.setHigh(i, bounds_pre_base_area[i][1]);
    }
    state_space_base_area->setBounds(bounds_base_area);  // bounds for param

    nav_msgs::msg::OccupancyGrid space_shapes = map;
    //TODO : robotの形状を追加する

    _space_info_base_area = std::make_shared<ob::SpaceInformation>(state_space_base_area);
    _space_info_base_area->setStateValidityChecker(std::make_shared<BaseArea>(_space_info_base_area, space_shapes));  // checker
    _space_info_base_area->setMotionValidator(std::make_shared<BaseAreaMotionValidator>(_space_info_base_area, space_shapes));  // motion validator
    _space_info_base_area->setup();
  }

  nav_msgs::msg::Path OMPL_PlannerClass::plan(const geometry_msgs::msg::Pose &start_state,const geometry_msgs::msg::Pose &goal_state){
    auto state_space(std::make_shared<ob::RealVectorStateSpace>(2));
    auto space_info = _space_info_base_area;

    // start point
    ob::ScopedState<> start(state_space);
    double yaw, pitch, roll;
    tf2::getEulerYPR(start_state.orientation, yaw, pitch,roll);
    std::cout << "start: x,y,theta -> " << start_state.position.x << ", " << start_state.position.y << ", " << yaw << std::endl;
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_state.position.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_state.position.y;


    // goal point
    ob::ScopedState<> goal(state_space);
    tf2::getEulerYPR(goal_state.orientation, yaw, pitch,roll);
    std::cout << "goal: x,y,theta -> " << goal_state.position.x << ", " << goal_state.position.y << ", " << yaw << std::endl;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_state.position.x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_state.position.y;

    // problem definition
    auto prob_def(std::make_shared<ob::ProblemDefinition>(space_info));  // problem instance
    prob_def->setStartAndGoalStates(start, goal);
    prob_def->setOptimizationObjective(getPathLengthObjective(space_info));  // 目的関数の設定
    //    prob_def->print(std::cout);  // 問題設定を表示

    auto planner = allocatePlanner(space_info, PLANNER_INF_RRTSTAR);

    // planner settings
    double start_goal_dist = sqrt(pow(start_state.position.x - goal_state.position.x, 2.0) + pow(start_state.position.y - goal_state.position.y, 2.0));
    planner.get()->as<og::InformedRRTstar>()->setGoalBias(0.1);
    planner.get()->as<og::InformedRRTstar>()->setRange(start_goal_dist/4.0*3.0);

    planner->setProblemDefinition(prob_def);  // problem instanceを代入
    planner->setup();  // plannerのsetup
    planner->checkValidity();

    ob::PlannerStatus solved = planner->ob::Planner::solve(0.002);
    if (!solved) {
        std::cout << "No solution found" << std::endl;
        auto null_path = nav_msgs::msg::Path();
        return null_path;
    }

    auto path = std::static_pointer_cast<og::PathGeometric>(prob_def->getSolutionPath());
    if(path->getStates().size() <= 2){  // スプライン補間は3点以上必要
        path->interpolate(3);
    }
    path->checkAndRepair(5);

    auto traj = nav_msgs::msg::Path();
    for(auto& tmp: path->getStates()){
        auto [x, y] = get2DPos(tmp);
        auto pose_stamped = geometry_msgs::msg::PoseStamped();
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        traj.poses.push_back(pose_stamped);
    }
    return traj;
  }
} // namespace global_path
#include "global_path/global_planning.hpp"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

template<class T> using matrix= std::vector<std::vector<T>>;

namespace global_path{
  // ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr space_info, PlannerType plannerType){
  //   switch (plannerType){
  //       case PLANNER_BFMTSTAR:
  //           return std::make_shared<og::BFMT>(space_info);
  //           break;
  //       case PLANNER_CFOREST:
  //           return std::make_shared<og::CForest>(space_info);
  //           break;

  //       case PLANNER_FMTSTAR:
  //           return std::make_shared<og::FMT>(space_info);
  //           break;

  //       case PLANNER_INF_RRTSTAR:
  //           return std::make_shared<og::InformedRRTstar>(space_info);
  //           break;
  //       case PLANNER_PRMSTAR:
  //           return std::make_shared<og::PRMstar>(space_info);
  //           break;
  //       case PLANNER_RRTSTAR:
  //           return std::make_shared<og::RRTstar>(space_info);
  //           break;
  //       case PLANNER_SORRTSTAR:
  //           return std::make_shared<og::SORRTstar>(space_info);
  //           break;
  //       default:
  //           OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
  //           return ob::PlannerPtr(); // Address compiler warning re: no return value.
  //           break;
  //   }
  // }

  OMPL_PlannerClass::OMPL_PlannerClass(){
  //   auto state_space_base_area = std::make_shared<ob::RealVectorStateSpace>(2);
  //   matrix<double> bounds_pre_base_area{  // 元のstateの範囲
  //           {-100, 100},
  //           {-30, 30}
  //   };
  //   ob::RealVectorBounds bounds_base_area(2);
  //   for(int i=0; i<bounds_pre_base_area.size(); i++){
  //       bounds_base_area.setLow(i, bounds_pre_base_area[i][0]);
  //       bounds_base_area.setHigh(i, bounds_pre_base_area[i][1]);
  //   }
  //   state_space_base_area->setBounds(bounds_base_area);  // bounds for param

  //   std::vector<std::shared_ptr<SpaceShapes>> space_shapes = {  // fieldの定義
  //           std::shared_ptr<SpaceShapes>(new RectangleSpace({-100, -30}, {100, 30}, false)),
  //           std::shared_ptr<SpaceShapes>(new RectangleSpace({2, 2}, {4, 4}, true)),
  //           std::shared_ptr<SpaceShapes>(new CircleSpace({7, 7}, 2, true)),
  //   };  // space shapes

  //   _space_info_base_area = std::make_shared<ob::SpaceInformation>(state_space_base_area);
  //   _space_info_base_area->setStateValidityChecker(std::make_shared<BaseArea>(_space_info_base_area, space_shapes));  // checker
  //   _space_info_base_area->setMotionValidator(std::make_shared<BaseAreaMotionValidator>(_space_info_base_area, space_shapes));  // motion validator
  //   _space_info_base_area->setup();
  // }

  // std::vector<RobotState> OMPL_PlannerClass::plan(const RobotState& start_state, const RobotState& goal_state){
  //   auto state_space(std::make_shared<ob::RealVectorStateSpace>(2));
  //   auto space_info = _space_info_base_area;

  //   // start point
  //   ob::ScopedState<> start(state_space);
  //   std::cout << "start: x,y,theta -> " << start_state.x << ", " << start_state.y << ", " << start_state.theta << std::endl;
  //   start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_state.x;
  //   start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_state.y;


  //   // goal point
  //   ob::ScopedState<> goal(state_space);
  //   std::cout << "goal: x,y,theta -> " << goal_state.x << ", " << goal_state.y << ", " << goal_state.theta << std::endl;
  //   goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_state.x;
  //   goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_state.y;

  //   // problem definition
  //   auto prob_def(std::make_shared<ob::ProblemDefinition>(space_info));  // problem instance
  //   prob_def->setStartAndGoalStates(start, goal);
  //   prob_def->setOptimizationObjective(getPathLengthObjective(space_info));  // 目的関数の設定
  //   //    prob_def->print(std::cout);  // 問題設定を表示

  //   auto planner = allocatePlanner(space_info, PLANNER_INF_RRTSTAR);

  //   // planner settings
  //   double start_goal_dist = length({start_state.x, start_state.y}, {goal_state.x, goal_state.y});
  //   planner.get()->as<og::InformedRRTstar>()->setGoalBias(0.1);
  //   planner.get()->as<og::InformedRRTstar>()->setRange(start_goal_dist/4.0*3.0);

  //   planner->setProblemDefinition(prob_def);  // problem instanceを代入
  //   planner->setup();  // plannerのsetup
  //   planner->checkValidity();

  //   ob::PlannerStatus solved = planner->ob::Planner::solve(0.002);
  //   if (!solved) {
  //       std::cout << "No solution found" << std::endl;
  //       return std::vector<RobotState>{};
  //   }

  //   auto path = std::static_pointer_cast<og::PathGeometric>(prob_def->getSolutionPath());
  //   if(path->getStates().size() <= 2){  // スプライン補間は3点以上必要
  //       path->interpolate(3);
  //   }
  //   path->checkAndRepair(5);

  //   std::vector<RobotState> traj;
  //   for(auto& tmp: path->getStates()){
  //       auto [x, y] = get2DPos(tmp);
  //       traj.emplace_back(x, y, 0.0);
  //   }
  //   return traj;
  }
} // namespace global_path
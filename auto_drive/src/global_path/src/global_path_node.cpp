#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "global_path/global_planning.hpp"


using namespace std::chrono_literals;

class GlobalPathNode : public rclcpp::Node {
public:
  GlobalPathNode() : Node("global_path_node") {

    // configure parameters
    declare_parameter("path_dir", "./global_path.csv");
    declare_parameter("start_x", 0.0);
    declare_parameter("start_y", 0.0);
    declare_parameter("goal_x", 0.0);
    declare_parameter("goal_y", 0.0);
    declare_parameter("robot_size", 0.0);
    get_parameter("path_dir", path_dir);
    get_parameter("start_x", start_x);
    get_parameter("start_y", start_y);
    get_parameter("goal_x", goal_x);
    get_parameter("goal_y", goal_y);
    get_parameter("robot_size", robot_size);
    std::cout << "path_dir: " << path_dir << std::endl;
    std::cout << "start_x: " << start_x << std::endl;
    std::cout << "start_y: " << start_y << std::endl;
    std::cout << "goal_x: " << goal_x << std::endl;
    std::cout << "goal_y: " << goal_y << std::endl;
    std::cout << "robot_size: " << robot_size << std::endl;

    publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 1);

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map = *msg;

        // path planning
        auto start = geometry_msgs::msg::Pose();
        start.position.x = start_x;
        start.position.y = start_y;
        start.orientation.z = 0.0;
        start.orientation.w = 1.0;

        auto goal = geometry_msgs::msg::Pose();
        goal.position.x = goal_x;
        goal.position.y = goal_y;
        goal.orientation.z = 0.0;
        goal.orientation.w = 1.0;

        global_path::OMPL_PlannerClass planner(map, robot_size);

        start_time = std::chrono::system_clock::now();
        nav_msgs::msg::Path global_path = planner.plan(start, goal);

        global_path.header.frame_id = "map";
        global_path.header.stamp = this->now();

        end_time = std::chrono::system_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count(); //処理に要した時間をミリ秒に変換
        std::cout << "planning time: " << elapsed << " ms" << std::endl;

        // save path
        std::ofstream ofs(path_dir);
        for(auto& pose: global_path.poses){
            ofs << pose.pose.position.x << "," << pose.pose.position.y << std::endl;
        }
        ofs.close();

        this->publisher_->publish(global_path);
      });
  }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

    std::chrono::system_clock::time_point start_time, end_time;
    nav_msgs::msg::OccupancyGrid map;
    std::string path_dir;
    double start_x, start_y, goal_x, goal_y, robot_size;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPathNode>());
    rclcpp::shutdown();
    return 0;
}
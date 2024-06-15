#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "global_path/global_planning.hpp"


using namespace std::chrono_literals;

class PathVisualizeNode : public rclcpp::Node {
public:
  PathVisualizeNode() : Node("path_visualize_node") {

    publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 1);

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map = *msg;

        auto start = geometry_msgs::msg::Pose();
        start.position.x = 0.0;
        start.position.y = 0.0;
        start.orientation.z = 0.0;
        start.orientation.w = 1.0;

        auto goal = geometry_msgs::msg::Pose();
        goal.position.x = 10.0;
        goal.position.y = 10.0;
        goal.orientation.z = 0.0;
        goal.orientation.w = 1.0;

        global_path::OMPL_PlannerClass planner(map);

        start_time = std::chrono::system_clock::now();
        nav_msgs::msg::Path global_path = planner.plan(start, goal);

        global_path.header.frame_id = "map";
        global_path.header.stamp = this->now();
        
        end_time = std::chrono::system_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count(); //処理に要した時間をミリ秒に変換
        std::cout << "planning time: " << elapsed << " ms" << std::endl;

        // pathを表示
        std::cout << "path size: " << global_path.poses.size() << std::endl;
        std::cout << "path: " << std::endl;
        for(auto& pose: global_path.poses){
            std::cout << "x: " << pose.pose.position.x << ", y: " << pose.pose.position.y << std::endl;
        }

        this->publisher_->publish(global_path);
      });
  }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

    std::chrono::system_clock::time_point start_time, end_time;
    nav_msgs::msg::OccupancyGrid map;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathVisualizeNode>());
    rclcpp::shutdown();
    return 0;
}
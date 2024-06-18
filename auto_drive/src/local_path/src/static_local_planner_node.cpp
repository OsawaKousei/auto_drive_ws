#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <spline.hpp>

#include <fstream>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "local_path/local_planning.hpp"
#include "local_path/util_functions.hpp"


using namespace std::chrono_literals;

class StaticLocalPlannerNode : public rclcpp::Node {
public:
  StaticLocalPlannerNode() : Node("static_local_planner_node") {

    // configure parameters
    declare_parameter("global_path_dir", "./global_path.csv");
    declare_parameter("local_path_dir", "./local_path.csv");
    get_parameter("global_path_dir", global_path_dir_);
    get_parameter("local_path_dir", local_path_dir_);
    std::cout << "path_dir: " << global_path_dir_ << std::endl;
    std::cout << "local_path_dir: " << local_path_dir_ << std::endl;

    // read path from csv file
    std::ifstream ifs(global_path_dir_);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file: %s", global_path_dir_.c_str());
      return;
    }

    std::string line;
    while (std::getline(ifs, line)) {
      std::istringstream iss(line);
      std::string token;
      std::vector<double> point;
      while (std::getline(iss, token, ',')) {
        point.push_back(std::stod(token));
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = point[0];
      pose.pose.position.y = point[1];
      pose.pose.position.z = 0.0;
      global_path_.poses.push_back(pose);
    }
    global_path_.header.frame_id = "map";
    local_path_.header.frame_id = "map";

    local_publisher_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 1);
    global_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);

    // timer
    timer = this->create_wall_timer(1000ms, std::bind(&StaticLocalPlannerNode::timer_callback, this));

    generate_local_path();
  }

  // timer
  void timer_callback() {
    mutex_.lock();
    global_path_.header.stamp = this->now();
    local_path_.header.stamp = this->now();
    this->local_publisher_->publish(local_path_);
    this->global_publisher_->publish(global_path_);
    mutex_.unlock();
  }

  

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_publisher_;

    std::chrono::system_clock::time_point start_time, end_time;
    std::string global_path_dir_;
    std::string local_path_dir_;
    nav_msgs::msg::Path global_path_;
    nav_msgs::msg::Path local_path_;

    //timer
    rclcpp::TimerBase::SharedPtr timer;

    std::vector<double> xs;
    std::vector<double> ys;

    std::mutex mutex_;

    void generate_local_path() {
      mutex_.lock();
      // generate local path
      xs.clear();
      ys.clear();

      for (auto& state : global_path_.poses) {
        xs.push_back(state.pose.position.x);
        ys.push_back(state.pose.position.y);
      }

      start_time = std::chrono::system_clock::now();
      auto [xs_new, ys_new] = spline_by_num(xs, ys, 300);  // スプライン補間
      end_time = std::chrono::system_clock::now();
      double elapsed_first = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count(); //処理に要した時間をミリ秒に変換

      start_time = std::chrono::system_clock::now();
      auto [xs_local, ys_local] = spline_by_min_max(xs_new, ys_new, 0.01, 0.15, 0.015);  // 台形加減速
      end_time = std::chrono::system_clock::now();
      double elapsed_second = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count();

      std::cout << "elapsed time for spline_by_num: " << elapsed_first << " ms" << std::endl;
      std::cout << "elapsed time for spline_by_min_max: " << elapsed_second << " ms" << std::endl;

      local_path_.poses.clear();
      local_path_.header = global_path_.header;
      for (int i = 0; i < int(xs_local.size()); i++){
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = xs_local[i];
        pose.pose.position.y = ys_local[i];
        pose.pose.position.z = 0.0;
        local_path_.poses.push_back(pose);
      }

      // save path
      std::ofstream ofs(local_path_dir_);
      for(auto& pose: local_path_.poses){
          ofs << pose.pose.position.x << "," << pose.pose.position.y << std::endl;
      }
      ofs.close();

      std::cout << "local path is saved to " << local_path_dir_ << std::endl;

      mutex_.unlock();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticLocalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
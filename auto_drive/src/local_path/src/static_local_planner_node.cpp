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
#include "util/pcp.hpp"
#include "util/rviz_util.hpp"


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
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("corner_marker", 1);

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

    // corner_pointsの座標を表示
    for(int i = 0; i < corner_points.size(); i++){
      auto viz_marker = viz_marker::std_cube_setter(corner_points[i]);
      viz_marker->header.frame_id = "map";
      viz_marker->header.stamp = this->now();
      viz_marker->id = i;

      marker_publisher_->publish(*viz_marker);

      std::cout << "i: " << i << ", x: " << std::get<0>(corner_points[i]) << ", y: " << std::get<1>(corner_points[i]) << std::endl;

      // 10ms待機
      std::this_thread::sleep_for(10ms);
    }
    mutex_.unlock();
  }

  

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    std::chrono::system_clock::time_point start_time, end_time;
    std::string global_path_dir_;
    std::string local_path_dir_;
    nav_msgs::msg::Path global_path_;
    nav_msgs::msg::Path local_path_;

    //timer
    rclcpp::TimerBase::SharedPtr timer;

    std::vector<double> xs;
    std::vector<double> ys;

    std::vector<std::tuple<double, double>> corner_points;

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
      auto [xs_new, ys_new] = spline_by_num(xs, ys, 1000);  // スプライン補間
      end_time = std::chrono::system_clock::now();
      double elapsed_first = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count(); //処理に要した時間をミリ秒に変換

      // start_time = std::chrono::system_clock::now();
      // auto [xs_local, ys_local] = spline_by_min_max(xs, ys, 0.01, 0.15, 0.015);  // 台形加減速
      // end_time = std::chrono::system_clock::now();
      // double elapsed_second = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count();

      // // std::cout << "elapsed time for spline_by_num: " << elapsed_first << " ms" << std::endl;
      // std::cout << "elapsed time for spline_by_min_max: " << elapsed_second << " ms" << std::endl;

      std::vector<std::vector<double>> path;
      for (int i = 0; i < int(xs_new.size()); i++) {
        path.push_back({xs_new[i], ys_new[i]});
      }

      auto corner_index = pcp::PCFeatureDetection::corner_detection(path);

      //手動でcorner_indexを修正
      // corner_indexの最初と２番目を削除
      corner_index.erase(corner_index.begin());
      corner_index.erase(corner_index.begin());

      // corner_indexの値を変更
      corner_index[0] = corner_index[0] - 10;
      corner_index[1] = corner_index[1] + 5;
      corner_index[2] = corner_index[2] + 10;
      corner_index[3] = corner_index[3];
      corner_index[4] = corner_index[4] - 5;
      corner_index[5] = corner_index[5] + 5;

      // corner_idex番目の点の座標を取得
      for (int i = 0; i < int(corner_index.size()); i++) {
        corner_points.push_back(std::make_tuple(xs_new[corner_index[i]], ys_new[corner_index[i]]));
      }

      local_path_.poses.clear();
      local_path_.header = global_path_.header;
      for (int i = 0; i < int(xs_new.size()); i++){
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = xs_new[i];
        pose.pose.position.y = ys_new[i];
        pose.pose.position.z = 0.0;
        local_path_.poses.push_back(pose);
      }

      // corner_indexを表示でpathを分割
      std::vector<nav_msgs::msg::Path> local_paths;
      nav_msgs::msg::Path local_path;
      local_path.header = global_path_.header;

      // 最初のpathを追加
      for (int i = 0; i < corner_index[0]; i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = xs_new[i];
        pose.pose.position.y = ys_new[i];
        pose.pose.position.z = 0.0;
        local_path.poses.push_back(pose);
      }
      local_paths.push_back(local_path);
      std::cout << "add first path" << std::endl;

      // cornerからcornerまでのpathを追加
      for (int i = 0; i < int(corner_index.size()); i++) {
        local_path.poses.clear();
        for (int j = corner_index[i]; j < corner_index[i + 1]; j++) {
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position.x = xs_new[j];
          pose.pose.position.y = ys_new[j];
          pose.pose.position.z = 0.0;
          local_path.poses.push_back(pose);
        }
        local_paths.push_back(local_path);

        std::cout << "add path from corner to corner : " << i << std::endl;
      }

      // local_pathsの最後のpathを削除
      local_paths.pop_back();

      // 最後のcornerから最後までのpathを追加
      local_path.poses.clear();
      for (int i = corner_index.back(); i < int(xs_new.size()); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = xs_new[i];
        pose.pose.position.y = ys_new[i];
        pose.pose.position.z = 0.0;
        local_path.poses.push_back(pose);
      }
      local_paths.push_back(local_path);
      std::cout << "add last path" << std::endl;

      // 分割されたpathを表示
      for (int i = 0; i < int(local_paths.size()); i++) {
        std::cout << "local_paths[" << i << "]: " << local_paths[i].poses.size() << std::endl;
      }

      // 分割したpathに対して台形加減速を適用
      for (int i = 0; i < int(local_paths.size()); i++) {
        xs.clear();
        ys.clear();

        for (auto& state : local_paths[i].poses) {
          xs.push_back(state.pose.position.x);
          ys.push_back(state.pose.position.y);
        }

        auto [xs_local, ys_local] = spline_by_min_max(xs, ys, 0.01, 0.15, 0.015);
        local_paths[i].poses.clear();
        for (int j = 0; j < int(xs_local.size()); j++) {
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position.x = xs_local[j];
          pose.pose.position.y = ys_local[j];
          pose.pose.position.z = 0.0;
          local_paths[i].poses.push_back(pose);
        }
      }

      // 分割したpathをlocal_pathに追加
      local_path_.poses.clear();
      for (int i = 0; i < int(local_paths.size()); i++) {
        for (int j = 0; j < int(local_paths[i].poses.size()); j++) {
          local_path_.poses.push_back(local_paths[i].poses[j]);
        }
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
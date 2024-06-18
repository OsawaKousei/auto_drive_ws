#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <fstream>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "visualization_msgs/msg/marker.hpp"

#include "local_path/path_publisher.hpp"
#include "util/pcp.hpp"
#include "util/rviz_util.hpp"

using namespace std::chrono_literals;
namespace local_path {

class CornerDetectNode : public rclcpp::Node {
public:
  CornerDetectNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("local_planner", options) {

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "global_path", 1,
        [this](const nav_msgs::msg::Path::SharedPtr path) { path_callback(path); });

    corner_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("corner", 1);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("corner_marker", 1);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  std::mutex mutex_;

  void path_callback(const nav_msgs::msg::Path::SharedPtr path) {
    mutex_.lock();

    // pick up the first 10 points
    // nav_msgs::msg::Path path_10;
    // for (int i = 0; i < 10; i++) {
    //   path_10.poses.push_back(path->poses[i]);
    // }

    //path_10のｙ座標を表示
    // for (int i = 0; i < 10; i++) {
    //   std::cout << "y: " << path_10.poses[i].pose.position.y << std::endl;
    // }

    auto corner_pc2 = pcp::PCConvert::path2pc2(*path);
    corner_pc2->header.frame_id = "map";
    corner_pc2->header.stamp = this->now();
    corner_publisher_->publish(*corner_pc2);

    // coner_pc2のｙ座標を表示
    // float *data_ptr = reinterpret_cast<float *>(corner_pc2->data.data());
    // for (int i = 0; i < int(corner_pc2->width); i++) {
    //   std::cout << "y: " << data_ptr[i * 3 + 1] << std::endl;
    // }

    // pcp::PCFeatureDetection pc(corner_pc2);
    // auto [x, y, pv] = pc.PCA();

    // std::cout << "x: " << x << ", y: " << y << ", pv: " << pv << std::endl;

    // auto viz_marker = viz_marker::std_line_setter(std::make_tuple(0.0, 0.0), std::make_tuple(x, y));
    // viz_marker->header.frame_id = "map";
    // viz_marker->header.stamp = this->now();
    // std::cout << "publish line from (0, 0) to (" << x << ", " << y << ")" << std::endl;
    // marker_publisher_->publish(*viz_marker);

    pcp::PCFeatureDetection pc(corner_pc2);
    auto corner_index = pc.corner_detection();

    // corner_idex番目の点の座標を取得
    std::vector<std::tuple<double, double>> corner_points;
    float *data_ptr = reinterpret_cast<float *>(corner_pc2->data.data());
    for (int i = 0; i < int(corner_index.size()); i++) {
      corner_points.push_back(std::make_tuple(data_ptr[corner_index[i] * 3], data_ptr[corner_index[i] * 3 + 1]));
    }

    // corner_pointsの座標を表示
    for(int i = 0; i < int(corner_points.size()); i++){
      auto viz_marker = viz_marker::std_cube_setter(corner_points[i]);
      viz_marker->header.frame_id = "map";
      viz_marker->header.stamp = this->now();
      viz_marker->id = i;

      marker_publisher_->publish(*viz_marker);
    }
    
    mutex_.unlock();
  }

};
  
} // namespace local_path

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto coner_detect =
      std::make_shared<local_path::CornerDetectNode>(rclcpp::NodeOptions());
  exec.add_node(coner_detect);

  const auto path_publisher =
      std::make_shared<local_path::PathPublisher>(rclcpp::NodeOptions());
  exec.add_node(path_publisher);
  
  exec.spin();
  rclcpp::shutdown();
}
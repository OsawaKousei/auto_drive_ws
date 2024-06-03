/**
 * @file save_pc.cpp
 * @brief Implementation file for the SavePc class
 * @author kousei
 * @date 2024-05-29
 */
#include "localization_dev/pc_mapping/save_pc.hpp"
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace localization_dev {

SavePc::SavePc(const rclcpp::NodeOptions &options)
    : rclcpp::Node("save_pc", options) {
  declare_parameter("map_dir", "default");
  get_parameter("map_dir", map_dir);
  // configure parameters
  std::cout << "map_dir: " << map_dir << std::endl;

  current_map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "mapped_pc2", 1,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        map_callback(msg);
      });
  pc_mapping_switch_sub = this->create_subscription<std_msgs::msg::String>(
      "pc_mapping_cmd", 1, [this](const std_msgs::msg::String::SharedPtr msg) {
        switch_callback(msg);
      });
}

// デストラクタ
SavePc::~SavePc() {}

void SavePc::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  mutex_.lock();
  pcl::fromROSMsg(*msg, current_map_pc);
  mutex_.unlock();
}

void SavePc::switch_callback(const std_msgs::msg::String::SharedPtr msg) {
  mutex_.lock();
  std::cout << "Received command: " << msg->data << std::endl;
  if (msg->data == "save_map") {
    save_pc(current_map_pc);
  }
  mutex_.unlock();
}

void SavePc::save_pc(pcl::PointCloud<pcl::PointXYZ> cloud) {
  // get project path
  std::string file_dir;
  std::string file_name;
  // set file name as current time
  std::time_t now = std::time(nullptr);
  file_name = std::to_string(now);
  file_dir = map_dir + file_name + ".pcd";

  std::cout << "file_dir: " << file_dir << std::endl;

  try {
    pcl::io::savePCDFileASCII(file_dir, cloud);
    std::cout << "Saved map point cloud to " << file_dir << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }
}

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::SavePc)
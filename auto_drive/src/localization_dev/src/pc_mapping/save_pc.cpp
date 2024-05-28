#include "localization_dev/pc_mapping/save_pc.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>
#include <iostream>
#include <filesystem>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

using namespace std::chrono_literals;
using namespace ament_index_cpp;
namespace fs = std::filesystem;


namespace localization_dev
{

SavePc::SavePc(const rclcpp::NodeOptions & options)
: rclcpp::Node("save_pc", options)
{
    declare_parameter("pkg_path", "default");
    get_parameter("pkg_path", pkg_path);
    // configure parameters
    std::cout << "pkg_path: " << pkg_path << std::endl;

    current_map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "mapped_pc2", 1, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { map_callback(msg); });
    pc_mapping_switch_sub = this->create_subscription<std_msgs::msg::String>(
        "pc_mapping_cmd", 1, [this](const std_msgs::msg::String::SharedPtr msg) { switch_callback(msg); });
}

// デストラクタ
SavePc::~SavePc()
{
}

void SavePc::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::fromROSMsg(*msg, current_map_pc);
}

void SavePc::switch_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::cout << "Received command: " << msg->data << std::endl;
    if (msg->data == "save_map")
    {
        save_pc(current_map_pc);
    }
}

void SavePc::save_pc(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    // get project path
    std::string file_dir;
    file_dir = pkg_path + "/map/"+ "map.pcd";

    std::cout << "file_dir: " << file_dir << std::endl;
    if (fs::exists(pkg_path) && fs::is_directory(pkg_path)) {
        std::cout << "ディレクトリが存在します。" << std::endl;
    } else {
        std::cout << "ディレクトリは存在しません。" << std::endl;
    }

    try{
        pcl::io::savePCDFileASCII(file_dir, cloud);
        std::cout << "Saved map point cloud to " << file_dir << std::endl;
    } catch (const std::exception &e){
        std::cerr << e.what() << std::endl;
    }
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::SavePc)
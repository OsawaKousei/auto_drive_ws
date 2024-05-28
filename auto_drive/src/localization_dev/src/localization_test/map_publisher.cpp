#include "localization_dev/localization_test/map_publisher.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;


namespace localization_dev
{

MapPublisher::MapPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_publisher", options)
{
    declare_parameter("map_dir", "default");
    declare_parameter("map_name", "default");
    get_parameter("map_dir", map_dir);
    get_parameter("map_name", map_name);
    // configure parameters
    std::cout << "map_dir: " << map_dir << std::endl;
    std::cout << "map_name: " << map_name << std::endl;
    map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MapPublisher::timer_callback, this));
}

// デストラクタ
MapPublisher::~MapPublisher()
{
}

void MapPublisher::timer_callback()
{
    sensor_msgs::msg::PointCloud2 msg;

    // read pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    try{
        pcl::io::loadPCDFile<pcl::PointXYZ>(map_dir + "/" + map_name, *cloud);
    } catch (std::runtime_error e){
        std::cerr << "Error in reading pcd file: " << e.what() << std::endl;
    
    }
    
    pcl::toROSMsg(*cloud, msg);

    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    map_pub->publish(msg);
}
}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::MapPublisher)
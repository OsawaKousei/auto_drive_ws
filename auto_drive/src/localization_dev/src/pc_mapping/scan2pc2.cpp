#include "localization_dev/pc_mapping/scan2pc2.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>
#include <iostream>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;


namespace localization_dev
{

Scan2pc2::Scan2pc2(const rclcpp::NodeOptions & options)
: rclcpp::Node("scan2pc2", options)
{
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, [this](const sensor_msgs::msg::LaserScan &scan) {
            // convert LaserScan to PointCloud2
            sensor_msgs::msg::PointCloud2 pc2;

            pc2.header = scan.header;
            pc2.height = 1;
            pc2.width = scan.ranges.size();
            pc2.fields.resize(3);
            pc2.fields[0].name = "x";
            pc2.fields[0].offset = 0;
            pc2.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            pc2.fields[0].count = 1;
            pc2.fields[1].name = "y";
            pc2.fields[1].offset = 4;
            pc2.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            pc2.fields[1].count = 1;
            pc2.fields[2].name = "z";
            pc2.fields[2].offset = 8;
            pc2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            pc2.fields[2].count = 1;
            pc2.is_bigendian = false;
            pc2.point_step = 12;
            pc2.row_step = 12 * scan.ranges.size();
            pc2.is_dense = true;
            pc2.data.resize(pc2.point_step * pc2.width);
            float *data = reinterpret_cast<float *>(pc2.data.data());
            for (int i = 0; i < scan.ranges.size(); i++) {
                data[i * 3] = scan.ranges[i] * cos(scan.angle_min + i * scan.angle_increment);
                data[i * 3 + 1] = scan.ranges[i] * sin(scan.angle_min + i * scan.angle_increment);
                data[i * 3 + 2] = 0.0;
            }

            pc2_pub->publish(pc2);
        });
    pc2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("raw_pc2", 10);
}

// デストラクタ
Scan2pc2::~Scan2pc2()
{
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Scan2pc2)
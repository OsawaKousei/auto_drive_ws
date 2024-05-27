#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std::chrono_literals;

class PclTestNode : public rclcpp::Node {
public:
    PclTestNode() : Node("pcl_test_node") {
        pc2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pc2", 10);

        auto topic_callback = [this](const sensor_msgs::msg::PointCloud2 &msg) -> void {
            auto message = sensor_msgs::msg::PointCloud2();

            this->pc2_pub->publish(message);
        }; 

        pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("raw_pc2", 10, topic_callback);
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclTestNode>());
    rclcpp::shutdown();
    return 0;
}
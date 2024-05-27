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
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(msg, *cloud);

            RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",msg.height,msg.width);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

            // PassThrough Filter
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");  // x axis
            // extract point cloud between 1.0 and 3.0 m
            pass.setFilterLimits(1.0,3.0);
            // pass.setFilterLimitsNegative (true);   // extract range reverse
            pass.filter(*cloud_filtered);

            // // Voxel Grid: pattern 1
            // pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
            // voxelGrid.setInputCloud(cloud);
            // float leaf_size_ = 0.1;
            // // set the leaf size (x, y, z)
            // voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            // // apply the filter to dereferenced cloudVoxel
            // voxelGrid.filter(*cloud_filtered);

            // // Radius Outlier Removal
            // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            // outrem.setInputCloud(cloud);
            // outrem.setRadiusSearch(0.1);
            // outrem.setMinNeighborsInRadius(2);
            // outrem.setKeepOrganized(true);
            // outrem.filter(*cloud_filtered);

            // publish filtered point cloud
            auto message = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(*cloud_filtered, message);
            message.header.frame_id = "map";
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
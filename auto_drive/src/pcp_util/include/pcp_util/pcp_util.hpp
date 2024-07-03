#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcp {

class PCFeatureDetection {
public:
    explicit PCFeatureDetection(sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    PCFeatureDetection(const pcl::PointCloud<pcl::PointXY> &cloud);
    ~PCFeatureDetection();

    std::tuple<double,double,double> PCA();
    static std::tuple<double, double, double> PCA(const std::vector<std::vector<double>> &data);
    std::vector<int> corner_detection();
    static std::vector<int> corner_detection(const std::vector<std::vector<double>> &cloud);
private:
    pcl::PointCloud<pcl::PointXY> cloud_;
};

namespace convert{
    // utility functions to convert between different types
    sensor_msgs::msg::PointCloud2 path2pc2(const nav_msgs::msg::Path::SharedPtr path);
    sensor_msgs::msg::PointCloud2 scan2pc2(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    std::vector<std::vector<double>> pc2matrix(const pcl::PointCloud<pcl::PointXY> &cloud);
    pcl::PointCloud<pcl::PointXY> pcxyz2xy(const pcl::PointCloud<pcl::PointXYZ> &cloud);
} // namespace convert
} // namespace pcp
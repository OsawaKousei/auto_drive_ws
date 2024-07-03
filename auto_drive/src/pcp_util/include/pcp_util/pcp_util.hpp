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
    explicit PCFeatureDetection(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    PCFeatureDetection(const pcl::PointCloud<pcl::PointXY> &cloud);
    ~PCFeatureDetection();

    std::tuple<double,double,double> PCA();
    static std::tuple<double, double, double> PCA(const std::vector<std::vector<double>> &data);
    static std::vector<std::vector<int>> near_threshold_clsutering(const std::vector<int> &index, const int near_threshold);
    static std::vector<int> min_threshold_filter(const std::vector<double> &index, const double threshold);

    void set_corner_params(const int window_size, const double pv_threshold, const int near_threshold);
    std::vector<int> corner_detection();
    static std::vector<int> corner_detection(const std::vector<std::vector<double>> &data, const int window_size, const double pv_threshold, const int near_threshold);

private:
    pcl::PointCloud<pcl::PointXY> cloud_;
    int coner_window_size_ = 5;
    double coner_pv_threshold_ = 0.999995;
    int corner_near_threshold_ = 17;
};

class Localization {
public:
    explicit Localization(const sensor_msgs::msg::PointCloud2::SharedPtr current_cloud, const sensor_msgs::msg::PointCloud2::SharedPtr base_cloud);
    Localization(const pcl::PointCloud<pcl::PointXY> &current_cloud, const pcl::PointCloud<pcl::PointXY> &base_cloud);
    ~Localization();

    std::vector<std::vector<double>> feature_matching();
    std::vector<std::vector<double>> localization(const std::vector<double> &odom);

private:
    pcl::PointCloud<pcl::PointXY> current_cloud_;
    pcl::PointCloud<pcl::PointXY> base_cloud_;
};

namespace convert{
    // utility functions to convert between different types
    sensor_msgs::msg::PointCloud2 path2pc2(const nav_msgs::msg::Path::SharedPtr path);
    sensor_msgs::msg::PointCloud2 scan2pc2(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    std::vector<std::vector<double>> pc2matrix(const pcl::PointCloud<pcl::PointXY> &cloud);
    pcl::PointCloud<pcl::PointXY> pcxyz2xy(const pcl::PointCloud<pcl::PointXYZ> &cloud);
} // namespace convert
} // namespace pcp
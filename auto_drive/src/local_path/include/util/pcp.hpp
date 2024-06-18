#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
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
    PCFeatureDetection(pcl::PointCloud<pcl::PointXY> cloud);
    ~PCFeatureDetection();

    std::tuple<double,double,double> PCA();
    std::vector<int> corner_detection();
private:
    pcl::PointCloud<pcl::PointXY> cloud_;
};

class PCConvert{
public:
    PCConvert();
    ~PCConvert();

    static sensor_msgs::msg::PointCloud2::SharedPtr path2pc2(nav_msgs::msg::Path path);
};

} // namespace pcp
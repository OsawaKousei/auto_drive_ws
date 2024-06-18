#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>

namespace pcp {

class PCFeatureDetection {
public:
    explicit PCFeatureDetection(sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    ~PCFeatureDetection();

    std::tuple<double,double,double> PCA();
    std::vector<int> corner_detection();
private:
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_;
};

class PCConvert{
public:
    PCConvert();
    ~PCConvert();

    static sensor_msgs::msg::PointCloud2::SharedPtr path2pc2(nav_msgs::msg::Path path);
};

} // namespace pcp
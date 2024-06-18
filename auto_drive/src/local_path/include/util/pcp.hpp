#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

class PCFeatureDetection {
public:
    explicit PCFeatureDetection(sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    ~PCFeatureDetection();

private:
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_;

    std::tuple<double,double,double> PCA();
    std::vector<int> corner_detection();
};
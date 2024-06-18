#include "util/pcp.hpp"

namespace pcp {
  PCFeatureDetection::PCFeatureDetection(sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    : cloud_(cloud) {}
  PCFeatureDetection::~PCFeatureDetection() {}

  sensor_msgs::msg::PointCloud2::SharedPtr PCConvert::path2pc2(nav_msgs::msg::Path path){
    sensor_msgs::msg::PointCloud2 pc2;

    pc2.header = path.header;
    pc2.height = 1;
    pc2.width = path.poses.size();
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
    pc2.row_step = 12 * path.poses.size();
    pc2.is_dense = true;
    pc2.data.resize(pc2.point_step * pc2.width);
    float *data = reinterpret_cast<float *>(pc2.data.data());
    for (int i = 0; i < path.poses.size(); i++) {
      data[i * 3] = path.poses[i].pose.position.x;
      data[i * 3 + 1] = path.poses[i].pose.position.y;
      data[i * 3 + 2] = 0.0;
    }

    return std::make_shared<sensor_msgs::msg::PointCloud2>(pc2);
  }
} // namespace pcp
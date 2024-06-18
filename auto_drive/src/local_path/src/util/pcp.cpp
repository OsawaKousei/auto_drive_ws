#include "util/pcp.hpp"
#include <Eigen/Core> // Include the necessary header file
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcp.hpp"

namespace pcp {
  PCFeatureDetection::PCFeatureDetection(sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    : cloud_(pcl::PointCloud<pcl::PointXY>()){
      this->cloud_ = pcl::PointCloud<pcl::PointXY>();
      pcl::fromROSMsg(*cloud, this->cloud_);

    // float *data_ptr = reinterpret_cast<float *>(cloud_->data.data());
    // for (int i = 0; i < int(cloud_->width); i++) {
    //   std::cout << "y: " << data_ptr[i * 3 + 1] << std::endl;
    // }
    }

  PCFeatureDetection::PCFeatureDetection(pcl::PointCloud<pcl::PointXY> cloud)
    : cloud_(cloud){
    this->cloud_ = cloud;
  }
  PCFeatureDetection::~PCFeatureDetection() {}

  std::tuple<double,double,double> PCFeatureDetection::PCA(){
    // PCA
    // float *data_ptrs = reinterpret_cast<float *>(cloud_->data.data());
    // for (int i = 0; i < int(cloud_->width); i++) {
    //   std::cout << "y: " << data_ptrs[i * 3 + 1] << std::endl;
    // }
    Eigen::MatrixXd data(cloud_.size(), 2);
    for (int i = 0; i < int(cloud_.size()); i++) {
      data(i, 0) = cloud_[i].x;
      data(i, 1) = cloud_[i].y;
    }
    //平均を表示
    // std::cout << "mean: " << data.colwise().mean() << std::endl;
    // 平均を0にする
    Eigen::MatrixXd centered = data.rowwise() - data.colwise().mean();
    // std::cout << "centered: " << centered << std::endl;
    // 標準偏差を表示
    // std::cout << "std: " << data.colwise().norm().array() << std::endl;
    // 標準偏差を1にする
    centered.array().rowwise() /= data.colwise().norm().array();
    // std::cout << "centered: " << centered << std::endl;
    // 共分散行列を求める
    Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(data.rows() - 1);
    // std::cout << "cov: " << cov << std::endl;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    Eigen::Vector2d eigenvalues = eig.eigenvalues();
    Eigen::Matrix2d eigenvectors = eig.eigenvectors();

    // std::cout << "eigenvalues: " << eigenvalues << std::endl;
    // std::cout << "eigenvectors: " << eigenvectors << std::endl;

    double x = eigenvectors(0, 0);
    double y = eigenvectors(1, 0);

    //calculate the Proportion of Variance
    double sum = eigenvalues.sum();
    double pv = eigenvalues(0) / sum;

    // return x: eigenvector x, y: eigenvector y, pv: Proportion of Variance
    return std::make_tuple(x, y, pv);
  }

  std::vector<int> PCFeatureDetection::corner_detection(){
    const int window_size = 5; //奇数である必要あり
    std::vector<int> pv_index;
    double pv_threshold = 100;
    std::vector<int> corner_index;
    // corner detection

    // pv_index: Proportion of Varianceを格納するvector
    // pv_indexのwindow_size番目までを1で初期化
    for(int i = 0; i < int((window_size+1)/2 -1); i++){
      pv_index.push_back(pv_threshold + 1);
    }
    //pv_indexを表示
    // for(int i = 0; i < int(pv_index.size()); i++){
    //   std::cout << "pv_index[" << i << "]: " << pv_index[i] << std::endl;
    // }

    // window_sizeの範囲でPCAを行い、Proportion of Varianceをpv_indexに追加
    for(int i = 0; i < cloud_.size() - window_size; i++){
      pcl::PointCloud<pcl::PointXY>::Ptr window_pc2(new pcl::PointCloud<pcl::PointXY>);
      window_pc2->resize(window_size);
      for (int j = 0; j < window_size; j++) {
        window_pc2->points[j].x = cloud_[i + j].x;
        window_pc2->points[j].y = cloud_[i + j].y;
        
        // std::cout << "window_pc2->points[" << j << "].x: " << window_pc2->points[j].x << std::endl;
        // std::cout << "window_pc2->points[" << j << "].y: " << window_pc2->points[j].y << std::endl;
      }
      // std::cout << "window_pc2->data.size(): " << window_pc2->data.size() << std::endl;
      PCFeatureDetection pc(*window_pc2);
      auto [x, y, pv] = pc.PCA();
      // std::cout << "x: " << x << ", y: " << y << ", pv: " << pv << std::endl;
      pv = pv * 1e+8;
      pv_index.push_back(pv);
    }

    // pv_indexを表示
    for(int i = 0; i < int(pv_index.size()); i++){
      std::cout << "pv_index[" << i << "]: " << pv_index[i] << std::endl;
    }

    // pv_indexの中でProportion of Varianceがpv_threshold以下のindexをcorner_indexに追加
    // ただし、前後のindexのpvが自身のpvより小さい場合は追加しない
    for(int i = 0; i < int(pv_index.size()); i++){
      if(pv_index[i] < pv_threshold){
        if(pv_index[i] < pv_index[i - 1] && pv_index[i] < pv_index[i + 1]){
          corner_index.push_back(i);
        }
      }
    }

    // corner_indexを表示
    for(int i = 0; i < int(corner_index.size()); i++){
      // std::cout << "corner_index[" << i << "]: " << corner_index[i] << std::endl;
    }

    return corner_index;
  }

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
    for (int i = 0; i < int(path.poses.size()); i++) {
      data[i * 3] = path.poses[i].pose.position.x;
      data[i * 3 + 1] = path.poses[i].pose.position.y;
      data[i * 3 + 2] = 0.0;
    }

    return std::make_shared<sensor_msgs::msg::PointCloud2>(pc2);
  }
} // namespace pcp
#include <Eigen/Core> // Include the necessary header file
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcp_util/pcp_util.hpp"

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

  PCFeatureDetection::PCFeatureDetection(const pcl::PointCloud<pcl::PointXY> &cloud)
    : cloud_(cloud){
    this->cloud_ = cloud;
  }

  PCFeatureDetection::~PCFeatureDetection() {}

  std::tuple<double,double,double> PCFeatureDetection::PCA(const std::vector<std::vector<double>> &data){
    // PCA
    Eigen::MatrixXd data_matrix(data.size(), data[0].size());
    for (int i = 0; i < int(data.size()); i++) {
      for (int j = 0; j < int(data[0].size()); j++) {
        data_matrix(i, j) = data[i][j];
      }
    }

    // std::cout << "data_matrix: " << std::endl << data_matrix << std::endl;
    Eigen::MatrixXd centered = data_matrix.rowwise() - data_matrix.colwise().mean();
    centered.array().rowwise() /= data_matrix.colwise().norm().array();
    Eigen::MatrixXd cov = (data_matrix.adjoint() * data_matrix) / double(data_matrix.rows());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    Eigen::Vector2d eigenvalues = eig.eigenvalues();
    Eigen::Matrix2d eigenvectors = eig.eigenvectors();

    // std::cout << "eigenvalues: " << std::endl << eigenvalues << std::endl;
    // std::cout << "eigenvectors: " << std::endl << eigenvectors << std::endl;

    double x = eigenvectors(0, 1);
    double y = eigenvectors(1, 1);

    double sum = eigenvalues.sum();
    double pv = eigenvalues(1) / sum;

    return std::make_tuple(x, y, pv);
  };

  std::tuple<double,double,double> PCFeatureDetection::PCA(){
    std::vector<std::vector<double>> data = convert::pc2matrix(cloud_);
    return PCA(data);
  }

  std::vector<std::vector<int>> PCFeatureDetection::near_threshold_clsutering(const std::vector<int> &index, const int near_threshold){
    std::vector<std::vector<int>> group_index;
    
    // indexの最後を除く各要素に対して検証
    for(int i = 0; i < int(index.size()) -1; i++){
      //自身が含まれるgropがあるかどうか
      bool is_grouped = false;
      int idx = 0;
      for(int j = 0; j < int(group_index.size()); j++){
        for(int k = 0; k < int(group_index[j].size()); k++){
          if(index[i] == group_index[j][k]){
            is_grouped = true;
            idx = j;
            break;
          }
        }
      }

      //自身が含まれるgroupがない場合、新しいgroupを作成
      if(!is_grouped){
        std::vector<int> new_group;
        new_group.push_back(index[i]);
        group_index.push_back(new_group);
        idx = group_index.size() - 1;
      }


      //index[i]の次のindexとの距離がnear_threshold以下の場合、同じgroupに追加
      if(index[i+1] - index[i] < near_threshold){
        group_index[idx].push_back(index[i+1]);
      }
    }

    // group_indexを表示
    // for(int i = 0; i < int(group_index.size()); i++){
    //   for(int j = 0; j < int(group_index[i].size()); j++){
    //     std::cout << "group_corner_index[" << i << "][" << j << "]: " << group_index[i][j] << std::endl;
    //   }
    // }

    return group_index;
  }

  std::vector<int> PCFeatureDetection::min_threshold_filter(const std::vector<double> &index, const double threshold){
    std::vector<int> filtered_index;
    for(int i = 0; i < int(index.size() - 1); i++){
      if(index[i] < index[i+1] && index[i] < index[i-1] && index[i] < threshold){
        filtered_index.push_back(i);
      }
    }
    return filtered_index;
  }

  void PCFeatureDetection::set_corner_params(int window_size, double pv_threshold, int near_threshold){
    if(window_size < 1){
      throw std::invalid_argument("window_size must be greater than 1");
    }else if(window_size % 2 == 0){
      throw std::invalid_argument("window_size must be odd number");
    }else{
      this->coner_window_size_ = window_size;
    }
    if(pv_threshold < 0 || pv_threshold > 1){
      throw std::invalid_argument("pv_threshold must be between 0 and 1");
    }else{
      this->coner_pv_threshold_ = pv_threshold;
    }
    if(near_threshold < 1){
      throw std::invalid_argument("near_threshold must be greater than 1");
    }else{
      this->corner_near_threshold_ = near_threshold;
    }
  }

  std::vector<int> PCFeatureDetection::corner_detection(){
    const int window_size = this->coner_window_size_;
    const double pv_threshold = this->coner_pv_threshold_;
    const int near_threshold = this->corner_near_threshold_;

    std::vector<std::vector<double>> data = convert::pc2matrix(cloud_);
    return corner_detection(data, window_size, pv_threshold, near_threshold);
  }

  std::vector<int> PCFeatureDetection::corner_detection(const std::vector<std::vector<double>> &data, 
    const int window_size = 5, const double pv_threshold = 0.999995, const int near_threshold = 17
  ){
    std::vector<double> pv_index; // pv_index: Proportion of Varianceを格納するvector

    // pv_indexの(window_size+1)/2番目までを1で初期化
    for(int i = 0; i < int((window_size+1)/2 -1); i++){
      pv_index.push_back(pv_threshold + 1);
    }
    //pv_indexを表示
    // for(int i = 0; i < int(pv_index.size()); i++){
    //   std::cout << "pv_index[" << i << "]: " << pv_index[i] << std::endl;
    // }

    // data内の点をwindow_sizeの範囲でスライドさせながらPCAを行い、Proportion of Varianceをpv_indexに追加
    for(int i = 0; i < int(data.size()) - window_size; i++){
      std::vector<std::vector<double>> window_data;
      for (int j = 0; j < window_size; j++) {
        window_data.push_back(data[i + j]);
        
        // std::cout << "window_pc2->points[" << j << "].x: " << window_pc2->points[j].x << std::endl;
        // std::cout << "window_pc2->points[" << j << "].y: " << window_pc2->points[j].y << std::endl;
      }
      // std::cout << "window_pc2->data.size(): " << window_pc2->data.size() << std::endl;
      auto [x, y, pv] = PCA(window_data);
      // std::cout << "x: " << x << ", y: " << y << ", pv: " << pv << std::endl;
      pv_index.push_back(pv);
    }

    // pv_indexを表示
    // for(int i = 0; i < int(pv_index.size()); i++){
    //   std::cout << "pv_index[" << i << "]: " << pv_index[i] << std::endl;
    // }

    // pv_indexの中で極小値かつ閾値以下の点をcorner_indexに追加
    auto corner_index = min_threshold_filter(pv_index, pv_threshold);

    // corner_indexを表示
    // for(int i = 0; i < int(corner_index.size()); i++){
    //   std::cout << "corner_index[" << i << "]: " << corner_index[i]  << "  :  " << "pv_index[" << corner_index[i] << "]: " << pv_index[corner_index[i]] << std::endl;
    // }

    // corner_indexをnear_threshold_clsuteringにかけてgroup_corner_indexに分類
    auto group_corner_index = PCFeatureDetection::near_threshold_clsutering(corner_index, near_threshold);

    // group_corner_indexの各groupの中心をclassified_corner_indexに追加
    std::vector<int> classified_corner_index;
    for(int i = 0; i < int(group_corner_index.size()); i++){
      int sum = 0;
      for(int j = 0; j < int(group_corner_index[i].size()); j++){
        sum += group_corner_index[i][j];
      }
      classified_corner_index.push_back(sum / group_corner_index[i].size());
    }

    // corner_indexを表示
    // for(int i = 0; i < int(classified_corner_index.size()); i++){
    //   std::cout << "classified_corner_index[" << i << "]: " << classified_corner_index[i]  << "  :  " << "pv_index[" << classified_corner_index[i] << "]: " << pv_index[classified_corner_index[i]] << std::endl;
    // }

    return classified_corner_index;
  }

  sensor_msgs::msg::PointCloud2 convert::path2pc2(const nav_msgs::msg::Path::SharedPtr path)
  {
    sensor_msgs::msg::PointCloud2 pc2;

    pc2.header = path->header;
    pc2.height = 1;
    pc2.width = path->poses.size();
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
    pc2.row_step = 12 * path->poses.size();
    pc2.is_dense = true;
    pc2.data.resize(pc2.point_step * pc2.width);
    float *data = reinterpret_cast<float *>(pc2.data.data());
    for (int i = 0; i < int(path->poses.size()); i++) {
      data[i * 3] = path->poses[i].pose.position.x;
      data[i * 3 + 1] = path->poses[i].pose.position.y;
      data[i * 3 + 2] = 0.0;
    }

    return pc2;
  }

  std::vector<std::vector<double>> convert::pc2matrix(const pcl::PointCloud<pcl::PointXY> &cloud)
  {
    std::vector<std::vector<double>> data;
    for (int i = 0; i < int(cloud.width); i++) {
      std::vector<double> point;
      point.push_back(cloud.points[i].x);
      point.push_back(cloud.points[i].y);
      data.push_back(point);
    }

    return data;
  }

  pcl::PointCloud<pcl::PointXY> convert::pcxyz2xy(const pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
    pcl::PointCloud<pcl::PointXY> cloud_xy;
    cloud_xy.width = cloud.width;
    cloud_xy.height = 0;
    cloud_xy.is_dense = cloud.is_dense;
    cloud_xy.points.resize(cloud.points.size());
    for (int i = 0; i < int(cloud.points.size()); i++) {
      cloud_xy.points[i].x = cloud.points[i].x;
      cloud_xy.points[i].y = cloud.points[i].y;
    }

    return cloud_xy;
  }

  
} // namespace pcp
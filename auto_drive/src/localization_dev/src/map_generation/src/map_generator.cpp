#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

int main(int argc, char * argv[])
{
    // load pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../meshes/output.pcd", *cloud);

    // PassThrough Filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");  //z axis
    pass.setFilterLimits(0.04,1.0); // filter limits
    pass.filter(*cloud);

    // set each point's z to 0
    for (size_t i = 0; i < cloud->points.size(); i++){
        cloud->points[i].z = 0;
    }

    // Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50); // number of neighbors to analyze for each point
    sor.setStddevMulThresh(0.5); // standard deviation multiplier
    sor.setNegative(false);
    sor.filter (*cloud);

    // Voxel Grid Filter
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    float leaf_size_ = 0.075; // leaf size of the grid
    voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxelGrid.filter(*cloud);

    // save pcd file
    pcl::io::savePCDFileASCII("../map/map.pcd", *cloud);
}
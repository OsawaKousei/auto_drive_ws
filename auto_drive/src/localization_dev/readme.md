# localization development environment package

# requrement
- holonomic_sim package
- do `colcon build` and `source ./install/setup.bash`

## usage
### implement localization logic
- user have to implement estimate_pose() function in /src/localization_test/localization.cpp
- available information is lidar scan, odometry (add some noise), mapp data (pointcloud or occupancy grid)

    user can get these information by acces private member of Localization

massege type:

lidar scan  : sensor_msgs::msg::LaserScan

odometry    : nav_msgs::msg::Odometry

map         : sensor_msgs::msg::PointCloud2 and nav_msgs::msg::OccupancyGrid

- estimation result must be geometry_msgs::msg::Pose

    x,y and z for robot roation (rad)

    x axis is 0 rad and increase z (rotation) when rotate right direction

## localization test
- use this cmd`ros2 launch localization_dev localization_test.launch.py`
- move anywhere you like in simulator using teleop keyboad on simulator GUI or using RQT publisher
- push "localize" button to call estimate_pose() function you implemented

    real pose in simulator and estimated pose, estimate accuracy are displayed on xterm console

    localization accuracy is calculated as 1 / ( 1 + cbrt((x_error)^2 + (y_error)^2 + (z_error)^2)) in local_accuracy.cpp

## mapping
- simulation use map.pcd as default in /localization_dev/map/
    you can create new mapp by using this package function

at first, you should make point cloud map as .pcd format, then you can transrate it to occupancy grid

### manual point cloud mapping
- use this cmd`ros2 launch localization_dev pc_mapping_test.launch.py`
- move carefully, then map is automatically updated

    mapp is updated based on lidar scan and odometry

- push "save map" button to save created point cloud as .pcd

    this function implemented in pc_mapping_node

### generate point cloud map
- you can generate point cloud map from various CAD format file

    see readme.md in localization_dev/src/map_generation/ for more detail

### transrate point cloud map to occupancy grid
- both localization_test.launch.py and pc_mapping_test.launch.py transrate point cloud topic (/mapped_pc2) to occupancy grid (/map) automatically
- this function is implemented in pc2octmap.cpp in /src/pc_mapping

    you can change grid map size and resolution, etc by changing parameter in pc2octmap.cpp
- if you want to save occupancy grid map, use octmap_publisher package

    see readme.md in the package for details

## field
- simulation field base on HaruRobo2024field.stl as default in /holonomic_sim/models/field/meshes/

    you can use any field you like change it
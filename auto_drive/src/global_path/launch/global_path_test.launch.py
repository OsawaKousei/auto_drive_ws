import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro

def generate_launch_description():

    # launch configuration variables
    autostart = LaunchConfiguration('autostart', default='true') # setteing for lifecycle_manager

    # get package directory
    pkg_share_dir = get_package_share_directory('global_path')


    # get rviz config file path
    rviz_config_dir = os.path.join(
        pkg_share_dir,
        'config',
        'path_visualization.rviz'
    )
    
    # launch rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # specify the rviz config file
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
    
    # get map file path
    map_file_path = os.path.join(
        pkg_share_dir,
        'maps',
        'map.yaml'
    )

    # launch map_server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        # specify the map file
        parameters=[{'yaml_filename': map_file_path}]
    )

    # launch lifecycle_manager for map_server
    map_lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'autostart': autostart},
                    {'node_names': ['map_server']}],
        prefix="bash -c 'sleep 1; $0 $@' " # wait for amcl to start
    )
    
    # launch path_visualize_node
    path_visualize_node = Node(
        package='global_path',
        executable='global_path_node',
        output='screen',
        # prefix to open a new terminal
        # specify the ros2 parameters file
        parameters=[os.path.join(pkg_share_dir,'params','params.yaml')],
        prefix="xterm -e"
    )
    
    return LaunchDescription([
        rviz2,

        map_server,
        map_lifecycle_manager,

        path_visualize_node
    ])
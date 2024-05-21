import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='test_world')
    pkg_share_dir = get_package_share_directory('model_test')
    model_path = os.path.join(pkg_share_dir, "models")

    #ignition gazeboがモデルにアクセスできるように設定
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        model_path])

    #ロボットをスポーンさせる設定
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-entity', 'SingleSteerRobo',
                   '-name', 'SingleSteerRobo',
                   #ロボットのsdfファイルを指定
                   '-file', PathJoinSubstitution([
                        pkg_share_dir,
                        "models", "SingleSteerRobo", "model.sdf"]),
                    #ロボットの位置を指定
                   '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.0',
                   ],
        )
    
    #ワールドのsdfファイルを設定(worldタグのあるsdfファイル)
    world = os.path.join(pkg_share_dir,"worlds", "plugin_test.sdf")

    #ignition gazeboの起動設定
    ign_gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 3 ' +
                              world
                             ])])
    
    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ign_gz,
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),
    ])
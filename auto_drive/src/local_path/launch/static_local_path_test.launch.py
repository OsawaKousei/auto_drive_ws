import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro

def generate_launch_description():

    # launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true') # setteing for lifecycle_manager
    autostart = LaunchConfiguration('autostart', default='true') # setteing for lifecycle_manager
    world_name = LaunchConfiguration('world_name', default='test_world')

    # get package and model directory
    sim_pkg_dir = get_package_share_directory('holonomic_sim')
    model_path = os.path.join(sim_pkg_dir, "models")
    pkg_share_dir = get_package_share_directory('local_path')

    # Set ignition gazebo to have access to the model
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        model_path]
    )
    
    # spawn robot
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-entity', 'HolonomicRobo',
                   '-name', 'HolonomicRobo',
                   # specify the robot's urdf topic
                   '-topic', 'robot_description',
                   '-allow_renaming', 'true',
                   # specify the robot's initial pose
                   '-x', '0.5',
                   '-y', '0.5',
                   '-z', '1.0',
                   ]
    )
    
    # spawn field
    ignition_spawn_field = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        # specify the field's sdf file
        arguments=['-file', PathJoinSubstitution([
                        model_path, "field", "model.sdf"]),
                   '-allow_renaming', 'false',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',]
    )
    
    # get world file path
    world = os.path.join(model_path,"worlds", "localization_test.sdf")

    # lauch ignition gazebo
    ign_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                            'launch', 'ign_gazebo.launch.py')]),
        # specify the world file
        launch_arguments=[('ign_args', [' -r -v 3 ' +
                            world
                            ])]
    )
    
    # launch bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            # specify the bridge configuration file
            'config_file': os.path.join(pkg_share_dir, 'config', 'localization_test.yaml'),
            # QOS settings
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./odom.publisher.durability': 'transient_local',
        }],
        # remap the topics
        # remappings=[
        #     ("/odom", "/raw_odom"),
        # ],
        output='screen'
    )
    
    # get robot's urdf file path
    urdf = os.path.join(model_path, 'HolonomicUrdf','model.xacro') 
    # expand xacro
    robot_desc = xacro.process_file(urdf).toxml()

    # launch robot state publisher
    robot_state_publisher = Node( 
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='robot_state_publisher', 
        output='both', 
        arguments=[robot_desc], 
        # set the robot's urdf topic
        parameters=[{'robot_description': robot_desc}]
    )
    
    # tf between map and odom
    map_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )
    
    # get rviz config file path
    rviz_config_dir = os.path.join(
        pkg_share_dir,
        'config',
        'static_local_path_test.rviz'
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
    
    # launch rqt_publisher
    rqt = Node(
            package='rqt_publisher',
            executable='rqt_publisher',
            name='rqt_publisher',
            output='screen')
    
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
    
    # launch ignition debug node
    ign_debug = Node(
        package='holonomic_sim',
        executable='ign_ros_node',
        output='screen',
        # prefix to launch in a new terminal
        prefix="xterm -e"
    )

    # launch localization test node
    static_local_planner_node = Node(
        package='local_path',
        executable='static_local_planner_node',
        output='screen',
        # specify the ros2 parameters file
        parameters=[os.path.join(pkg_share_dir,'params','params.yaml')],
        # prefix to launch in a new terminal
        prefix="xterm -e"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        
        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),

        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_field,
        ign_gz,
                             
        bridge,

        robot_state_publisher,
        map_static_tf,
        rviz2,

        rqt,

        map_server,
        map_lifecycle_manager,

        # ign_debug,
        static_local_planner_node
    ])
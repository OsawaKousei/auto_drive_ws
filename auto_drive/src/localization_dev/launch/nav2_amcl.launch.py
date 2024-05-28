import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='test_world')
    #sim_pkg_dirにパッケージのパスを取得
    sim_pkg_dir = get_package_share_directory('holonomic_sim')
    #モデルの格納されているパスを設定
    model_path = os.path.join(sim_pkg_dir, "models")
    pkg_share_dir = get_package_share_directory('localization_dev')

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
        arguments=['-entity', 'HolonomicRobo',
                   '-name', 'HolonomicRobo',
                   #ロボットモデルを配信するトピックを指定
                   '-topic', 'robot_description',
                    #ロボットの位置を指定
                   '-allow_renaming', 'true',
                   '-x', '0.5',
                   '-y', '0.5',
                   '-z', '1.0',
                   ],
        )
    
    #フィールドをスポーンさせる設定
    ignition_spawn_field = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        #フィールドのsdfファイルを指定
        arguments=['-file', PathJoinSubstitution([
                        model_path, "field", "model.sdf"]),
                   '-allow_renaming', 'false',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',],
        )
    
    #ワールドのsdfファイルを設定(worldタグのあるsdfファイル)
    world = os.path.join(model_path,"worlds", "localization_test.sdf")

    #ignition gazeboの起動設定
    ign_gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            #worldファイルを指定
            launch_arguments=[('ign_args', [' -r -v 3 ' +
                              world
                             ])])
    
    #ros_ign_bridgeの起動設定
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            #brigdeの設定ファイルを指定
            'config_file': os.path.join(pkg_share_dir, 'config', 'localization_test.yaml'),
            #QoSの設定
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./odom.publisher.durability': 'transient_local',
        },{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    #ロボットのurdfファイルのパスを取得 
    urdf = os.path.join(model_path, 'HolonomicUrdf','model.xacro') 
    #xacroを展開
    robot_desc = xacro.process_file(urdf).toxml()

    #robot_state_publsherの起動設定 
    robot_state_publisher = Node( 
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher', 
            output='both', 
            arguments=[robot_desc], 
            #robot_descriptionのパラメータにロボットのurdfを設定
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': use_sim_time,}])
    
    #mapトピックとodomの関係を定義
    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.5', '0.5', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
                        parameters=[{'use_sim_time': use_sim_time}])
    
    #rviz2の設定フィルのパスを取得
    rviz_config_dir = os.path.join(
        pkg_share_dir,
        'config',
        'amcl.rviz')
    
    #rviz2の起動設定
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            #rvizの設定ファイルを指定
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    param_file_path = os.path.join(
        get_package_share_directory('localization_dev'),
        'params',
        'amcl_param.yaml'
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[param_file_path]
        )


    lifecycle_nodes = ['amcl']
    use_sim_time_ = True
    autostart = True

    start_lifecycle_manager = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time_},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    
    map_file_path = os.path.join(
        get_package_share_directory('octmap_publisher'),
        'maps',
        'map.yaml'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}])


    lifecycle_nodes = ['map_server']
    use_sim_time_ = True
    autostart = True

    start_lifecycle_manager_map = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time_},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}],
            prefix="bash -c 'sleep 5; $0 $@' ")
    
    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_field,
        ign_gz,
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        bridge,

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),
        
        robot_state_publisher,
        map_static_tf,
        rviz2,

        amcl,
        start_lifecycle_manager,

        map_server,
        start_lifecycle_manager_map
    ])
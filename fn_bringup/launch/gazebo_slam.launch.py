# Copyright (c) 2024.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():
    # 配置文件路径
    bringup_dir = get_package_share_directory('fine_nav2d_bringup')
    config_dir = os.path.join(bringup_dir, 'config')
    localization_manager_dir = FindPackageShare(package='localization_manager').find('localization_manager')
    odometry_gz_manager_dir = FindPackageShare(package='odometry_gz_manager').find('odometry_gz_manager')

    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Sim 模式 下的特殊设置
        description='Use simulation (Gazebo) clock if true'
    )

    declare_lidar_type = DeclareLaunchArgument(
        'lidar_type',
        default_value='virtual',
        description='Choose the type of lidar: livox or unitree'
    )

    declare_lio_type = DeclareLaunchArgument(
        'lio_type',
        default_value='fast_lio',
        description='Choose the type of LIO: fast_lio or point_lio'
    )

    declare_nav_mode = DeclareLaunchArgument(
        'navigation_mode',
        default_value='dynamic',
        description='Choose the mode of navigation: dynamic or static'
    )

    declare_map_save = DeclareLaunchArgument(
        'map_save',
        default_value='false',
        description='Enable octomap saving after navigation'
    )

    declare_map_load = DeclareLaunchArgument(
        'map_load',
        default_value='false',
        description='Enable octomap loading a map before navigation'
    )

    # 动态选择配置文件路径（使用 PythonExpression）
    fast_lio_config = PythonExpression([
        '"', config_dir, '/fastlio_mid360_config.yaml" if "', LaunchConfiguration('lidar_type'), '" == "livox" else ',
        '"', config_dir, '/fastlio_gazebo_config.yaml" if "', LaunchConfiguration('lidar_type'), '" == "virtual" else ',
        '"', config_dir, '/fastlio_unilidar_config.yaml"'
    ])
    

    # FAST-LIO 参数
    fast_lio_params = [
        fast_lio_config,
        {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }
    ]

    # FAST-LIO 节点
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        parameters=fast_lio_params,
        output='screen',
        condition=LaunchConfigurationEquals('lio_type', 'fast_lio')
    )

    # OctoMap 节点
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_node',
        output='both',
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resolution': 0.05,  # Resolution in meter for the map when starting with an empty map. Otherwise the loaded file's resolution is used.
            'frame_id': 'map',  # Static global frame in which the map will be published. A transform from sensor data to this frame needs to be available when dynamically building maps.
            'base_frame_id': 'base_link',  # The robot's base frame in which ground plane detection is performed (if enabled)
            'latch': False,  #  For maximum performance when building a map (with frequent updates), set to false. When set to true, on every map change all topics and visualizations will be created.
            'occupancy_min_z': 0.15,
            'occupancy_max_z': 0.5,
            'sensor_model.max_range': 3.0, # Maximum range in meter for inserting point cloud data when dynamically building a map.
            # 'point_cloud_min_z': 0.05,  # Minimum height of points to consider for insertion in the callback.
            'point_cloud_max_z': 2.45,  # Maximum height of points to consider for insertion in the callback.
            'filter_ground_plane': False,  # Whether the ground plane should be detected and ignored from scan data when dynamically building a map
            # 'ground_filter.distance': 0.4,  # Distance threshold for points (in z direction) to be segmented to the ground plane
            # 'ground_filter.angle': 0.15,  # Angular threshold of the detected plane from the horizontal plane to be detected as ground
            # 'ground_filter.plane_distance': 0.07,  # Distance threshold from z=0 for a plane to be detected as ground


        }],
        remappings=[('/cloud_in', '/cloud_registered_body')], # 要么区分LiDAR型号，要么规范话题名称
    )


    # 静态 TF 广播（odom → lidar_odom）这里做一个暂时的修改
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_to_lidar_odom',
    #     arguments=[
    #         "--x", "0.057", "--y", "0.083", "--z", "0.31",
    #         "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
    #         "--frame-id", "odom", "--child-frame-id", "lidar_odom"
    #     ]
    # )
        # 静态 TF 广播（odom → lidar_odom）这里做一个暂时的修改
    static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='odom_to_lidar_odom',
    arguments=[
        "--x", "0.0",   # X方向偏移：激光雷达在底盘前方0.8米
        "--y", "0.0",   # Y方向偏移：无左右偏移
        "--z", "0.85",   # Z方向偏移：激光雷达在底盘上方0.85米
        "--roll", "0.0",  # 无滚转偏移
        "--pitch", "0.0", 
        "--yaw", "0.0",  # 姿态无偏移（与底盘一致）
        "--frame-id", "odom", 
        "--child-frame-id", "lidar_odom"
    ],
    parameters=[{'use_sim_time': True}]
)

    # Localization Manager 节点
    localization_manager_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            localization_manager_dir, '/launch', '/example_gazebo.launch.py'
        ])
    )
    # Odometry gazebo manager 节点
    odometry_gz_manager_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            odometry_gz_manager_dir, '/launch', '/example_gazebo.launch.py'
        ])
    )

    # Map Manager 节点
    map_manager_node = Node(
        package='fn_map_manager',              # 包名
        executable='fn_map_manager_node',      # 可执行文件
        name='map_manager',                    # 节点名
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 启动顺序控制
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lidar_type)
    ld.add_action(declare_lio_type)
    # ld.add_action(declare_maplidar_TF_save)
    ld.add_action(declare_map_load)

    ld.add_action(declare_nav_mode)
    ld.add_action(static_tf_node)
    ld.add_action(fast_lio_node)
    # ld.add_action(map_manager_node)
    #ld.add_action(localization_manager_node)
    # ld.add_action(odometry_gz_manager_node)
    ld.add_action(TimerAction(period=5.0, actions=[octomap_server_node]))  # 延迟 5s 确保 FAST-LIO 初始化

    return ld
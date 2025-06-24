#  Copyright (c) 2024.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
import yaml
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():
    # 配置文件路径
    bringup_dir = get_package_share_directory('fine_nav2d_bringup')
    config_dir = os.path.join(bringup_dir, 'config')
    rviz_dir = os.path.join(bringup_dir, 'rviz')


    ################### Declare Parameters ###################
    # 定义参数
    # 1. 使用仿真时间
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use reality (Gazebo) clock if true'
    )
    # 2. 使用雷达的IMU还是下位机的IMU
    # 3. 可视化三维or二维
    # 4. 使用实际雷达or播放录制数据：播放三维点云 or 二维点云
    # 5. 可视化Lidar视角，决定安装位置
    # 6. 异步建图 or 同步建图
    # 7. 使用什么激光雷达
    declare_lidar_type = DeclareLaunchArgument(
        'lidar_type',
        default_value='livox',
        description='Choose the type of lidar: livox or unitree'
    )
    # 8. 启用录制
    declare_enable_recorder = DeclareLaunchArgument(
        'enable_recorder',
        default_value='false',
        description='Enable the recorder to record the data'
    )

    # 9. 用什么LIO
    declare_lio_type = DeclareLaunchArgument(
        'lio_type',
        default_value='fast_lio',
        description='Choose the type of LIO: fast_lio or point_lio'
    )

    # 10. 工作模式选择
    declare_working_mode = DeclareLaunchArgument(
        'working_mode',
        default_value='mapping',
        description='Choose the working mode: mapping, navigation, or exploration'
    )

    # 11. 是否可视化
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz to visualize the data'
    )

    # 12.串口编号
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',  # 默认值
        description='Serial port for fines_serial node'
    )


################### Robot Description ###################
    robot_description_dir = FindPackageShare(package='fines_description').find('fines_description')
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_dir, '/launch', '/publish_urdf.launch.py'])
    )


    ################### Serial Driver ###################
    serial_driver_node = Node(
        package = 'fines_serial',
        executable = 'fines_serial',
        name = 'serial_driver_node',
        arguments=[LaunchConfiguration('serial_port')],
        output = 'both',
    )

    ################### Livox Driver  ###################
    # Livox Mid 360
    livox_params = [
        {"xfer_format": 4},         # 0-PointCloud2Msg(PointXYZRTL), 1-LivoxCustomMsg, 2-PclPxyziMsg, 3-LivoxImuMsg, 4-AllMsg
        {"multi_topic": 0},         # 0-All LiDARs share the same topic, 1-One LiDAR one topic
        {"data_src": 0},            # 0-lidar, others-Invalid data src
        {"publish_freq": 20.0},     # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
        {"output_data_type": 0},    # 0-Output ROS topic, 1-Output ROS bag
        {"frame_id": 'base_lidar'},
        {"lvx_file_path": '/home/livox/livox_test.lvx'},
        {"user_config_path": os.path.join(config_dir, 'MID360_config.json')},
        {"cmdline_input_bd_code": 'livox0000000001'},
        {"self_filtering_box.min_x": -0.4},
        {"self_filtering_box.max_x": 0.3},
        {"self_filtering_box.min_y": -0.4},
        {"self_filtering_box.max_y": 0.25},
        {"self_filtering_box.min_z": -0.28},
        {"self_filtering_box.max_z": 0.05},
    ]

    livox_driver_node = Node(
        package = 'livox_ros_driver2',
        executable = 'livox_ros_driver2_node',
        name = 'livox_lidar_driver',
        output = 'both',
        parameters = livox_params,
        condition = LaunchConfigurationEquals('lidar_type', 'livox'),
        remappings = [
            ('/livox/lidar', '/lidar'),
            ('/livox/imu', '/imu'),
            ('/livox/lidar/pointcloud', '/lidar/pointcloud')
        ]
    )

    ################### Unitree Driver  ###################
    # Unitree L1
    unitree_params = [
        {'port': '/dev/unilidar'},
        {'rotate_yaw_bias': 0.0},
        {'range_scale': 0.001},
        {'range_bias': 0.0},
        {'range_max': 50.0},
        {'range_min': 0.0},
        {'cloud_frame': "base_lidar"},
        {'cloud_topic': "/lidar"},
        {'cloud_scan_num': 4}, # 4以及往下开始会有问题
        {'imu_frame': "base_lidar"},
        {'imu_topic': "/imu"},
        {'extrinsic_roll': -110.8},
        {'extrinsic_pitch': -62.3},
        {'extrinsic_yaw': -71.6},
        {"self_filtering_box.min_x": float('-inf')},
        {"self_filtering_box.max_x": 0.24},
        {"self_filtering_box.min_y": -0.5},
        {"self_filtering_box.max_y": 0.15},
        {"self_filtering_box.min_z": -0.50},
        {"self_filtering_box.max_z": 0.18}
    ]

    unitree_driver_node = Node(
        package = 'unitree_lidar_ros2',
        executable = 'unitree_lidar_ros2_node',
        name = 'unitree_lidar_driver',
        output = 'both',
        parameters = unitree_params,
        condition = LaunchConfigurationEquals('lidar_type', 'unitree')
    )

    ################### LIO ###################
    fast_lio_params = [
        os.path.join(config_dir, 'fastlio_mid360_config.yaml'), # 需要根据LiDAR型号选择
        {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }
    ]

    point_lio_params = [
        os.path.join(config_dir, 'pointlio_mid360_config.yaml'),
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3,  # Options: 1, 3
            'space_down_sample': True,
            'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
            'ivox_nearby_type': 6,   # Options: 0, 6, 18, 26
            'runtime_pos_log_enable': False,  # Option: True
        }
    ]

    localization_manager_dir = FindPackageShare(package='localization_manager').find('localization_manager')

    bringup_LIO_group = GroupAction([
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            name = 'odom_to_lidar_odom_broadcaster',
            arguments=[
                # Copied frome fines.urdf
                "--x", "0.0",
                "--y", "0.083",
                "--z", "0.31",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "odom",
                "--child-frame-id", "lidar_odom"
            ]
        ),
        Node(
            package = 'fast_lio',
            executable = 'fastlio_mapping',
            name = 'fast_lio',
            parameters = fast_lio_params,
            output = 'both',
            condition = LaunchConfigurationEquals('lio_type', 'fast_lio')
        ),
        Node(
            package = 'point_lio',
            executable = 'pointlio_mapping',
            name = 'point_lio',
            parameters = point_lio_params,
            output = 'both',
            condition = LaunchConfigurationEquals('lio_type', 'point_lio')
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([localization_manager_dir, '/launch', '/example.launch.py']))
    ])

    ################### PointCloud to Occupancy Grid ###################
    # http://wiki.ros.org/octomap_server
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_node',
        output='both',
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'use_sime_time': LaunchConfiguration('use_sim_time'),
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
        remappings=[('/cloud_in', '/lidar/pointcloud')], # 要么区分LiDAR型号，要么规范话题名称
    )

    ################### Nav2 ###################
    # work mode determine how nav2 and slamtoolbox work

    nav2_bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(bringup_dir,'maps','fins_bot_map.yaml'))
    nav2_param_path =LaunchConfiguration('params_file', default=os.path.join(bringup_dir,'config','nav2_real.yaml'))
    # rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_param_path}.items(),
    )


    ################### Recorder ###################
    recorder_config_path = os.path.join(config_dir, 'recorder_config.yaml')
    with open(recorder_config_path, 'r') as f:
        recorder_config = yaml.safe_load(f)

    recorder_topics = recorder_config.get('record_topics')
    recorder_save_path = os.path.join(
        recorder_config.get('bag_save_dir'),
        "FineNav-" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    )

    recorder_cmd = ['ros2', 'bag', 'record', '-o', recorder_save_path] + recorder_topics
    recorder_node = ExecuteProcess(
        cmd = recorder_cmd,
        output = 'screen',
        condition = IfCondition(LaunchConfiguration('enable_recorder'))
    )



    ################### Rviz2 ###################
    rviz_config_path = os.path.join(rviz_dir, 'FineNav.rviz')
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', rviz_config_path],
        condition = IfCondition(LaunchConfiguration('enable_rviz')),
    )


    ################### 启动顺序编排 ###################

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lidar_type)
    ld.add_action(declare_enable_recorder)
    ld.add_action(declare_lio_type)
    ld.add_action(declare_working_mode)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_serial_port)
    # 使用 TimerAction 来控制节点启动顺序 TODO PAREMETERS
    ld.add_action(TimerAction(period=0.0, actions=[recorder_node]))
    ld.add_action(TimerAction(period=0.0, actions=[robot_description]))
    ld.add_action(TimerAction(period=0.0, actions=[serial_driver_node]))
    ld.add_action(TimerAction(period=2.0, actions=[livox_driver_node]))
    ld.add_action(TimerAction(period=2.0, actions=[unitree_driver_node]))
    ld.add_action(TimerAction(period=6.0, actions=[bringup_LIO_group]))
    ld.add_action(TimerAction(period=8.0, actions=[nav2_bringup]))
    ld.add_action(TimerAction(period=8.0, actions=[rviz_node]))
    ld.add_action(TimerAction(period=10.0, actions=[octomap_server_node]))

    # ld.add_action(recorder_node)
    # ld.add_action(robot_description)
    # ld.add_action(serial_driver_node)
    # ld.add_action(livox_driver_node)
    # ld.add_action(unitree_driver_node)
    # # ld.add_action(filter_node)
    # ld.add_action(pointcloud_to_laserscan_node)
    # ld.add_action(bringup_LIO_group)
    # ld.add_action(start_mapping)
    # ld.add_action(nav2_bringup)
    # ld.add_action(rviz_node)

    return ld





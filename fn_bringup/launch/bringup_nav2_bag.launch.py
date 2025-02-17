#  Copyright (c) 2024.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.
########################################################
# Deprecated!!!
########################################################
import os
import yaml
from datetime import datetime
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from launch.substitutions import LocalSubstitution
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():
    # 配置文件路径
    bringup_dir = FindPackageShare(package='fine_nav_ros2').find('fine_nav_ros2')
    config_dir = os.path.join(bringup_dir, 'config')
    rviz_dir = os.path.join(bringup_dir, 'rviz')
    slam_toolbox_mapping_file_dir = os.path.join(bringup_dir, 'config',  'mapper_params_online_async_sim.yaml')
    map_saver_file_dir = os.path.join(bringup_dir, 'config',  'map_saver_param.yaml')
    ################### Declare Parameters ###################
    # 定义参数
    # 1. 使用仿真时间
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
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
    # 12. 是否保存地图
    declare_map_saver = DeclareLaunchArgument(
        'map_saver',
        default_value='false',
        description='Use map_saver if true'
    )
    # 13. 保存地图地址
    declare_map_saver_dir = DeclareLaunchArgument(
        'map_saver_dir',
        default_value=os.path.join(bringup_dir, 'maps', 'map.yaml'),
        description='the address of saved map'
    )

    ################### Robot Description ###################
    robot_description_dir = FindPackageShare(package='fines_description').find('fines_description')
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_dir, '/launch', '/publish_urdf.launch.py'])
    )

    ################### Livox Driver  ###################
    # Livox Mid 360
    livox_params = [
        {"xfer_format": 4},         # 0-PointCloud2Msg(PointXYZRTL), 1-LivoxCustomMsg, 2-PclPxyziMsg, 3-LivoxImuMsg, 4-AllMsg
        {"multi_topic": 0},         # 0-All LiDARs share the same topic, 1-One LiDAR one topic
        {"data_src": 0},            # 0-lidar, others-Invalid data src
        {"publish_freq": 10.0},     # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
        {"output_data_type": 0},    # 0-Output ROS topic, 1-Output ROS bag
        {"frame_id": 'base_lidar'},
        {"lvx_file_path": '/home/livox/livox_test.lvx'},
        {"user_config_path": os.path.join(config_dir, 'MID360_config.json')},
        {"cmdline_input_bd_code": 'livox0000000001'}
    ]

    ################### Unitree Driver  ###################
    # Unitree L1
    unitree_params = [
        {'port': '/dev/ttyUSB0'},
        {'rotate_yaw_bias': 0.0},
        {'range_scale': 0.001},
        {'range_bias': 0.0},
        {'range_max': 50.0},
        {'range_min': 0.0},
        {'cloud_frame': "lidar_frame"},
        {'cloud_topic': "/cloud"},
        {'cloud_scan_num': 18},
        {'imu_frame': "imu_frame"},
        {'imu_topic': "/imu"}
    ]

    ################### Serial Driver ###################



    ################### PointCloud Preprocess ###################

    filter_node = Node(
        package = 'cloud_filter',
        executable = 'filter_node',
        name = 'filter_node',
        output = 'screen',
        parameters = [{'visual': True}]
    )

    pointcloud_to_laserscan_node = Node(
        package = 'pointcloud_to_laserscan',
        executable = 'pointcloud_to_laserscan_node',
        name = 'pointcloud_to_laserscan_node',
        output = 'screen',
        parameters=[{
            'target_frame': 'base_link', # If provided, transform the pointcloud into this frame before converting to a laser scan.
            'transform_tolerance': 0.01, # Time tolerance for transform lookups. Only used if a target_frame is provided.
            'min_height': 0.01,
            'max_height': 0.4,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333, # The scan rate in seconds. Only used to populate the scan_time field of the output laser scan message.
            'range_min': 0.3, # unit: /m # 在不滤除机器人本体点云的情况下，设置为0.3m比较安全，这样车身探测障碍物距离大约在车前10cm左右
            'range_max': 10.0, # unit: /m
            'use_inf': True, # If disabled, report infinite range (no obstacle) as range_max + 1. Otherwise report infinite range as +inf.
            'inf_epsilon': 1.0
        }],
        remappings=[('cloud_in',  ['/lidar/pointcloud']),
                    ('scan',  ['/scan'])]
    )



    ################### LIO ###################
    fast_lio_params = [
        os.path.join(config_dir, 'fastlio_mid360_config.yaml'),
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

    bringup_LIO_group = GroupAction([
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            name = 'odom_to_lidar_odom_broadcaster',
            arguments=[
                # Copied frome fines.urdf
                "--x", "-0.0829998544383264",
                "--y", "0.0",
                "--z", "0.31",
                "--roll", "3.14159",    # Replace with actual calculated values
                "--pitch", "-0.1745",
                "--yaw", "1.67079",
                "--frame-id", "odom",
                "--child-frame-id", "lidar_odom"
            ]
        ),
        Node(
            package = 'fast_lio',
            executable = 'fastlio_mapping',
            name = 'fast_lio',
            parameters = fast_lio_params,
            output = 'screen',
            condition = LaunchConfigurationEquals('lio_type', 'fast_lio')
        ),
        Node(
            package = 'point_lio',
            executable = 'pointlio_mapping',
            name = 'point_lio',
            parameters = point_lio_params,
            output = 'screen',
            condition = LaunchConfigurationEquals('lio_type', 'point_lio')
        )
    ])
    ################### slam_toolbox ###################
    start_mapping = Node(
        condition = LaunchConfigurationEquals('working_mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': LaunchConfiguration('use_sim_time'),}
        ],
    )
    ################### map_saver ###################
    start_map_saver = Node(
        condition = LaunchConfigurationEquals('map_saver', 'true'),
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[map_saver_file_dir],
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

    ################### Rviz2 ###################
    rviz_config_path = os.path.join(rviz_dir, 'FineNav.rviz')
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', rviz_config_path],
        condition = IfCondition(LaunchConfiguration('enable_rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),}
        ],
    )

    ################### shutdown_action###################
    #on_shutdown_handler =  RegisterEventHandler(
    #    OnProcessExit(
    #        target_action=start_map_saver,
    #        on_exit=[
    #            LogInfo(msg=('close the map saver and save')),
    #            ExecuteProcess(
    #                cmd=[['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-t', 'map', '-f', '/fins/Desktop/fins_map'] ]
    #           )]
    #    )
    #)



    ld = LaunchDescription()


    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lidar_type)
    ld.add_action(declare_enable_recorder)
    ld.add_action(declare_lio_type)
    ld.add_action(declare_working_mode)
    ld.add_action(declare_enable_rviz)
    #ld.add_action(declare_map_saver_dir)
    #ld.add_action(declare_map_saver)


    # 使用 TimerAction 来控制节点启动顺序
    ld.add_action(TimerAction(period=0.0, actions=[robot_description]))
    ld.add_action(TimerAction(period=2.0, actions=[pointcloud_to_laserscan_node]))
    ld.add_action(TimerAction(period=6.0, actions=[bringup_LIO_group]))
    ld.add_action(TimerAction(period=15.0, actions=[start_mapping]))
    ld.add_action(TimerAction(period=15.0, actions=[start_map_saver]))
    ld.add_action(TimerAction(period=15.0, actions=[nav2_bringup]))
    ld.add_action(TimerAction(period=20.0, actions=[rviz_node]))
    # ld.add_action(recorder_node)
    # ld.add_action(robot_description)
    # # ld.add_action(filter_node)
    # ld.add_action(pointcloud_to_laserscan_node)
    # ld.add_action(bringup_LIO_group)
    # ld.add_action(start_mapping)
    # ld.add_action(nav2_bringup)
    # ld.add_action(rviz_node)

    return ld






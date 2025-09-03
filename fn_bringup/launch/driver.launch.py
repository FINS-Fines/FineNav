# Copyright (c) 2024.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 配置文件路径
    bringup_dir = get_package_share_directory('fine_nav2d_bringup')
    config_dir = os.path.join(bringup_dir, 'config')

    ################### 参数声明 ###################
    declare_lidar_type = DeclareLaunchArgument(
        'lidar_type',
        default_value='livox',
        description='Choose the type of lidar: livox or unitree'
    )

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for fines_serial node'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    ################### Robot Description (URDF) ###################
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('fines_description'),
            '/launch/publish_urdf.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    ################### 串口驱动 ###################
    serial_driver_node = Node(
        package='fines_serial',
        executable='fines_serial',
        name='serial_driver_node',
        arguments=[LaunchConfiguration('serial_port')],
        output='screen'
    )

    ################### Livox 雷达驱动 ###################
    livox_params = [
        {"xfer_format": 4},          # 0-PointCloud2, 1-CustomMsg, 4-AllMsg
        {"frame_id": 'base_lidar'},
        {"user_config_path": os.path.join(config_dir, 'MID360_config.json')},
        {"self_filtering_box.min_x": -0.4},
        {"self_filtering_box.max_x": 0.3},
        {"self_filtering_box.min_y": -0.4},
        {"self_filtering_box.max_y": 0.25},
        {"self_filtering_box.min_z": -0.28},
        {"self_filtering_box.max_z": 0.05},
    ]

    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_driver',
        parameters=livox_params,
        condition=LaunchConfigurationEquals('lidar_type', 'livox'),
        remappings=[
            ('/livox/lidar', '/lidar'),
            ('/livox/lidar/pointcloud', '/lidar/pointcloud'),
            ('/livox/imu', '/imu')
        ],
        output='screen'
    )

    ################### Unitree 雷达驱动 ###################
    unitree_params = [
        {'port': '/dev/unilidar'},
        {'cloud_frame': "base_lidar"},
        {'cloud_topic': "/lidar"},
        {'imu_topic': "/imu"},
        {"self_filtering_box.min_x": float('-inf')},
        {"self_filtering_box.max_x": 0.24},
        {"self_filtering_box.min_y": -0.5},
        {"self_filtering_box.max_y": 0.15},
        {"self_filtering_box.min_z": -0.50},
        {"self_filtering_box.max_z": 0.18}
    ]

    unitree_driver_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_driver',
        parameters=unitree_params,
        condition=LaunchConfigurationEquals('lidar_type', 'unitree'),
        output='screen'
    )

    ################### 启动描述 ###################
    ld = LaunchDescription()
    ld.add_action(declare_lidar_type)
    ld.add_action(declare_serial_port)
    ld.add_action(declare_use_sim_time)
    ld.add_action(robot_description_launch)
    ld.add_action(serial_driver_node)
    ld.add_action(livox_driver_node)
    ld.add_action(unitree_driver_node)

    return ld
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():
    # 配置文件路径

    bringup_dir = get_package_share_directory('fine_nav2d_bringup')
    config_dir = os.path.join(bringup_dir, 'config')
    rviz_dir = os.path.join(bringup_dir, 'rviz')
    launch_dir = os.path.join(bringup_dir, 'launch')


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
        default_value='/dev/ttyACM1',  # 默认值
        description='Serial port for fines_serial node'
    )

    declare_nav_mode = DeclareLaunchArgument(
        'navigation_mode',
        default_value='dynamic',
        description='Choose the mode of navigation: dynamic or static'
    )

    declare_nav_strategy = DeclareLaunchArgument(
        'navigation_strategy',
        default_value='aggressive',
        description='Choose the strategy of navigation: aggressive or conservative'
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

    launch_group = GroupAction([
        # 启动driver.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([launch_dir, 'driver.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'lidar_type': LaunchConfiguration('lidar_type'),
                'serial_port': LaunchConfiguration('serial_port'),
            }.items()
        ),

        # 启动slam.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([launch_dir, 'slam.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'lidar_type': LaunchConfiguration('lidar_type'),
                'lio_type': LaunchConfiguration('lio_type'),
                'navigation_mode': LaunchConfiguration('navigation_mode'),
                'map_save': LaunchConfiguration('map_save'),
                'map_load': LaunchConfiguration('map_load'),
            }.items()
        ),

        # 启动navigation.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([launch_dir, 'navigation.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_rviz': LaunchConfiguration('enable_rviz'),
                'navigation_strategy': LaunchConfiguration('navigation_strategy')
            }.items()
        )
    ])

    # # 启动foxglove_bridge
    # foxglove_dir = FindPackageShare(package='foxglove_bridge').find('foxglove_bridge')
    # foxglove_bridge = IncludeLaunchDescription(
    #     PathJoinSubstitution([foxglove_dir, 'launch', 'foxglove_bridge_launch.xml'])
    # )
    ################### 启动顺序编排 ###################

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lidar_type)

    ld.add_action(declare_lio_type)
    ld.add_action(declare_working_mode)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_serial_port)
    ld.add_action(declare_nav_mode)
    ld.add_action(declare_nav_strategy)
    ld.add_action(declare_map_save)
    ld.add_action(declare_map_load)

    ld.add_action(launch_group)
    #ld.add_action(foxglove_bridge)

    return ld





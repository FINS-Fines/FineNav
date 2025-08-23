#  Copyright (c) 2025.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.
# Copyright (c) 2025.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = get_package_share_directory('odometry_gz_manager')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # 这里声明使用虚拟的时间，在启动时使用虚拟时间
        description='Whether to use simulation time (from /clock topic)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    Odometry_gz_manager = Node(
        package='odometry_gz_manager',
        executable='Odometry_gz_manager_node',
        name='global_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(Odometry_gz_manager)


    return ld
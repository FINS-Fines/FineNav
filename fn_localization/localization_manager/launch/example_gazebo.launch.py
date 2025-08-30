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
    pkg_share = get_package_share_directory('localization_manager')
    param_file = os.path.join(pkg_share, 'config', 'example.yaml')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',  
        default_value='true',  # 这里声明使用虚拟的时间，在启动时使用虚拟时间
        description='Whether to use simulation time (from /clock topic)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    global_localization = Node(
        package='localization_manager',
        executable='localization_manager_node',
        name='global_localization',
        output='screen',
        parameters=[
            param_file, 
            {'use_sim_time': use_sim_time}  
        ],
    )

    local_localization = Node(
        package='localization_manager',
        executable='localization_manager_node',
        name='local_localization',
        output='screen',
        parameters=[
            param_file,
            {'use_sim_time': use_sim_time}  
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(global_localization)
    ld.add_action(local_localization)

    return ld
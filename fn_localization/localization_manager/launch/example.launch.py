#  Copyright (c) 2025.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('localization_manager')
    param_file = os.path.join(pkg_share, 'config', 'example.yaml')

    global_localization = Node(
        package='localization_manager',
        executable='localization_manager_node',
        name='global_localization',
        output='screen',
        parameters=[param_file],
    )

    local_localization = Node(
        package='localization_manager',
        executable='localization_manager_node',
        name='local_localization',
        output='screen',
        parameters=[param_file]
    )

    ld = LaunchDescription()
    ld.add_action(global_localization)
    ld.add_action(local_localization)

    return ld

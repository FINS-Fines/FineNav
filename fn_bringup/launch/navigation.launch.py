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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression

def generate_launch_description():
    # 配置文件路径
    bringup_dir = get_package_share_directory('fine_nav2d_bringup')
    rviz_dir = os.path.join(bringup_dir, 'rviz')
    config_dir = os.path.join(bringup_dir, 'config')

    ################### Declare Parameters ###################

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use reality (Gazebo) clock if true'
    )

    # 11. 是否可视化
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz to visualize the data'
    )

    declare_nav_strategy = DeclareLaunchArgument(
        'navigation_strategy',
        default_value='aggressive',
        description='Choose the strategy of navigation: aggressive or conservative'
    )

    declare_pct_planner_param = DeclareLaunchArgument(
        'pct_planner_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('fine_nav2d_bringup'),  # 自动查找包路径
            'config',
            'pct_planner.yaml'  # 默认参数文件
        ]),
        description='Full path to the Map Manager parameters file'
    )

    ################### Nav2 ###################
    # work mode determine how nav2 and slamtoolbox work

    nav2_bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(bringup_dir,'maps','fins_bot_map.yaml'))

    # 暂时修改为SIM的
    nav2_param_path = PythonExpression([
        '"', config_dir, '/nav2_', LaunchConfiguration('navigation_strategy'), '.yaml"'
    ])
    # rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir, '/launch', '/navigation_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_param_path}.items(),
    )
    ################### pct_node ###################
    pct_node = Node(
        package = 'fn_fine_pct',
        executable = 'fn_global_planner_node',
        parameters=[
            LaunchConfiguration('pct_planner_params_file')  # 使用动态参数
        ]
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

    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_nav_strategy)
    ld.add_action(declare_pct_planner_param)
    # 使用 TimerAction 来控制节点启动顺序 TODO PAREMETERS
    ld.add_action(pct_node)
    ld.add_action(TimerAction(period=8.0, actions=[nav2_bringup]))
    ld.add_action(TimerAction(period=8.0, actions=[rviz_node]))

    return ld





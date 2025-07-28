import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取当前包和点云工具包的路径
    pkg_gazebo_sim = get_package_share_directory('fines_gazebo_simulator')
    pkg_pointcloud_tool = get_package_share_directory('ign_sim_pointcloud_tool')
    
    # 世界文件路径（已包含机器人）
    world_sdf_path = os.path.join(pkg_gazebo_sim, 'resource','worlds', 'rmuc_2025_world.sdf')
    ign_config_path = os.path.join(pkg_gazebo_sim, 'resource', 'ign', 'gui.config')
    
    # 启动Gazebo Fortress并加载世界
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_sim, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
        }.items(),
    )

    # 包含机器人launch文件
    include_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_sim, "launch", "robot.launch.py")
        ),
    )

    # 包含点云处理节点的launch文件
    # 延迟3秒启动，确保Gazebo和机器人节点已初始化完成
    pointcloud_launch = TimerAction(
        period=2.0,  # 延迟3秒，给Gazebo足够的启动时间
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_pointcloud_tool, "launch", "pointcloud_complete_launch.py")
                )
            )
        ]
    )

    
    return LaunchDescription([
        gazebo_launch,
        include_robot_launch,
        pointcloud_launch  # 添加点云处理节点的启动
    ])
    
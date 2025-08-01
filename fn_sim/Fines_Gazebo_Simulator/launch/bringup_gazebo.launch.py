import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo_sim = get_package_share_directory('fines_gazebo_simulator')
    pkg_pointcloud_tool = get_package_share_directory('ign_sim_pointcloud_tool')
    
    world_sdf_path = os.path.join(pkg_gazebo_sim, 'resource','worlds', 'rmuc_2025_world.sdf')
    ign_config_path = os.path.join(pkg_gazebo_sim, 'resource', 'ign', 'gui.config')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_sim, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
        }.items(),
    )

    include_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_sim, "launch", "robot.launch.py")
        ),
    )

    pointcloud_launch = TimerAction(
        period=2.0,
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
        pointcloud_launch
    ])
    
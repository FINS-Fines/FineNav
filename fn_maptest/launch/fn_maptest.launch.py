from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    
    # 声明启动参数（可通过命令行传递）
    declare_pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='/home/fins/Desktop/Nav_ws/FineNav2D/fn_maptest/resource/cropped_5.pcd',
        description='PCD点云文件的绝对路径'
    )
    
    declare_topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='output_pointcloud',
        description='点云发布的话题名称'
    )
    
    declare_frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='点云的参考坐标系ID'
    )
    
    declare_publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='点云发布频率（Hz）'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Sim 模式 下的特殊设置
        description='Use simulation (Gazebo) clock if true'
    )


    

    # 定义pcd发布节点
    pcd_publisher_node = Node(
        package='fn_maptest',
        executable='pcd_publisher',
        name='pcd_publisher_node',
        output='screen',
        parameters=[
            {'pcd_file': LaunchConfiguration('pcd_file')},
            {'topic_name': LaunchConfiguration('topic_name')},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'publish_rate': LaunchConfiguration('publish_rate')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    

    # 构建启动描述
    return LaunchDescription([
        declare_pcd_file_arg,
        declare_topic_name_arg,
        declare_frame_id_arg,
        declare_publish_rate_arg,
        pcd_publisher_node,
        declare_use_sim_time,
    ])

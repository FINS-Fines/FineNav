from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():


    # TF坐标转换节点（依赖第一个节点的输出）
    # transformer_node = Node(
    #     package='ign_sim_pointcloud_tool',
    #     executable='tf_cloud_transformer_node',
    #     name='tf_cloud_transformer',
    #     output='screen',
    #     parameters=[
    #         {'target_frame': 'base_link'},  # 目标坐标系
    #         {'input_cloud_topic': '/livox/lidar'},  # 与第一个节点输出话题匹配livox/lidar
    #         {'output_cloud_topic': 'lidar_TF'},  # 最终输出话题
    #         {'tf_timeout': 0.1},
    #         {'base_lidar': 'base_lidar'}  # 激光雷达坐标系
    #     ]
    # )
    frame_id_change_node = Node(
        package='ign_sim_pointcloud_tool',
        executable='cloud_frame_id_changer_node',
        name='frame_id_changer',
        output='screen',
        parameters=[
            {'input_cloud_topic': '/livox/lidar'},  # 输入点云话题
            {'output_cloud_topic': 'lidar_TF'},  # 输出点云话题
            {'target_frame': 'base_lidar'}  # 新的frame_id
        ]
    )

        # 点云格式转换节点
    converter_node = Node(
        package='ign_sim_pointcloud_tool',
        executable='point_cloud_converter_node',
        name='point_cloud_converter',
        output='screen',
        parameters=[
            {'pcd_topic': 'lidar_TF'},  # 输入原始点云话题
            {'n_scan': 32},
            {'horizon_scan': 1875},
            {'ang_bottom': 7.0},
            {'ang_res_y': 1.0}
        ]
    )

    return LaunchDescription([
        # 先启动格式转换节点
        frame_id_change_node,
        converter_node
        # 延迟1秒启动坐标转换节点，确保第一个节点已初始化完成
        # TimerAction(
        #     period=1.0,
        #     actions=[transformer_node]
        # )
    ])
    
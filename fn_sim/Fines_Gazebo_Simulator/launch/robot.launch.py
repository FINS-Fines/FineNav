import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # 获取包路径（根据实际情况修改包名）
    pkg_simulator = get_package_share_directory("fines_gazebo_simulator")

    # ==========================
    # 配置你的SDF和URDF文件路径
    # ==========================
    sdf_path = os.path.join(
        pkg_simulator,
        "resource", 
        "robot",  # 可根据实际路径调整
        "model.sdf"  # 替换为你的SDF文件名
    )
    urdf_path = os.path.join(
        pkg_simulator,
        "urdf",  # 可根据实际路径调整
        "robot.urdf"  # 替换为你的URDF文件名
    )

    # 通信桥和机器人参数配置（保持不变）
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    # robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    # ==========================
    # 单个机器人参数（无需配置文件）
    # ==========================
    robot_name = "fine_robot"  # 机器人名称
    x_pose = "2.0"           # 初始X坐标
    y_pose = "2.0"           # 初始Y坐标
    z_pose = "0.5"           # 初始Z坐标
    yaw = "0.0"              # 初始偏航角

    # 读取SDF和URDF文件内容
    with open(sdf_path, 'r') as f:
        robot_xml = f.read()  # 直接读取SDF内容
    with open(urdf_path, 'r') as f:
        robot_urdf_xml = f.read()  # 直接读取URDF内容

    # 替换通信桥配置中的机器人名称
    aft_replace_ros_bridge_params = ReplaceString(
        source_file=bridge_config,
        replacements={"<robot_name>": robot_name},
    )

    ld = LaunchDescription()

    # 1. 在Gazebo中生成机器人
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_xml,
            "-name", robot_name,
            "-allow_renaming", "true",
            "-x", x_pose,
            "-y", y_pose,
            "-z", z_pose,
            "-Y", yaw,
        ],
    )

    # 3. 机器人状态发布器
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        remappings=remappings,
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_urdf_xml,
            }
        ],
    )

    # 4. ROS与Gazebo通信桥
    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=robot_name,
        parameters=[{"config_file": aft_replace_ros_bridge_params}],
    )


    # 添加所有节点到启动描述
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(robot_ign_bridge)

    return ld
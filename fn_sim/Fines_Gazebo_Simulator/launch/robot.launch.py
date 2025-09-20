import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("fines_gazebo_simulator")

    sdf_path = os.path.join(
        pkg_simulator,
        "resource", 
        "robot",  
        "model.sdf" 
    )
    urdf_path = os.path.join(
        pkg_simulator,
        "urdf",  
        "robot.urdf"  
    )

    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")

    robot_name = "fine_robot"  
    x_pose = "6.0"           
    y_pose = "-1.5"           
    z_pose = "0.8"           
    yaw = "0.0"             

    with open(sdf_path, 'r') as f:
        robot_xml = f.read() 
    with open(urdf_path, 'r') as f:
        robot_urdf_xml = f.read()  

    aft_replace_ros_bridge_params = ReplaceString(
        source_file=bridge_config,
        replacements={"<robot_name>": robot_name},
    )

    ld = LaunchDescription()

    # 1. 在Gazebo中生成机器人, 在指定位置生成机器人模型
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

    # 2. 机器人状态和关节发布器
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

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        remappings=remappings,
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_urdf_xml,
            }
        ],
        )

    # 3. ROS与Gazebo通信桥
    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=robot_name,
        parameters=[{"config_file": aft_replace_ros_bridge_params}],
    )

    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_ign_bridge)

    return ld
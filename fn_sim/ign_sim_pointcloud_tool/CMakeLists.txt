cmake_minimum_required(VERSION 3.10)
project(ign_sim_pointcloud_tool)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src  
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ign_sim_pointcloud_tool::PointCloudConverter 
  EXECUTABLE point_cloud_converter_node 
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ign_sim_pointcloud_tool::TfCloudTransformer 
  EXECUTABLE tf_cloud_transformer_node  
)

# 新增：注册 CloudFrameIdChanger 节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ign_sim_pointcloud_tool::CloudFrameIdChanger
  EXECUTABLE cloud_frame_id_changer_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
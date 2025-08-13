cmake_minimum_required(VERSION 3.8)
project(fn_fine_pct)

# 默认使用C++17
set(CMAKE_CXX_STANDARD 17)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)    # 用于 TransformStamped 消息

# 包含目录
include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

# 添加所有可执行文件
#add_executable(planner_node
#        src/planner/planner_node.cpp
#        src/planner/planner.cpp)

add_executable(tomography_node
        src/tomography/tomography_node.cpp
        src/tomography/tomography.cpp
        include/fn_fine_pct/Tomography.hpp)

# 链接库
#target_link_libraries(planner_node
#        ${rclcpp_LIBRARIES}
#        ${PCL_LIBRARIES}
#)

target_link_libraries(tomography_node
        ${rclcpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${PCL_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        ${tf2_ros_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        pcl_common
        pcl_io
)

#ament_target_dependencies(planner_node
#        rclcpp
#        PCL
#)

ament_target_dependencies(tomography_node
        rclcpp
        sensor_msgs
        pcl_conversions
        nav_msgs
        PCL
        visualization_msgs
        tf2_ros
        geometry_msgs
)

# 安装目标
install(TARGETS
        tomography_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
        DESTINATION include
)

# 安装资源文件
install(DIRECTORY rsc/
        DESTINATION share/${PROJECT_NAME}/rsc
)

# 导出依赖
ament_export_dependencies(
        rclcpp
        sensor_msgs
        nav_msgs
        visualization_msgs
        tf2_ros
        geometry_msgs
        PCL
)

ament_package()
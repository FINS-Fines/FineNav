cmake_minimum_required(VERSION 3.8)
project(fn_fine_pct)

# 默认使用C++17
set(CMAKE_CXX_STANDARD 17)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

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
        src/tomography/tomography.cpp)

# 链接库
#target_link_libraries(planner_node
#        ${rclcpp_LIBRARIES}
#        ${PCL_LIBRARIES}
#)

target_link_libraries(tomography_node
        ${rclcpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${PCL_LIBRARIES}
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
        PCL
)

# 安装目标
install(TARGETS
        # planner_node
        tomography_node
        DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
        DESTINATION include
)

# 安装资源文件
install(DIRECTORY rsc/
        DESTINATION share/${PROJECT_NAME}/rsc
)

ament_package()
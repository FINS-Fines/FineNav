cmake_minimum_required(VERSION 3.22)
project(odometry_gz_manager)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)  # 添加 tf2 库，确保能够使用 tf2::TimePointZero
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_runtime_c REQUIRED)


include_directories(
        include
        ${tf2_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
        ${tf2_geometry_msgs_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies()

ament_auto_add_library(Odometry_gz_manager SHARED
        src/Odometry_gz_manager.cpp
)

ament_auto_add_executable(Odometry_gz_manager_node
        src/Odometry_gz_manager_node.cpp
)

## 添加链接库的依赖
#ament_target_dependencies(localization_manager_node
#        rclcpp
#        tf2_ros
#        tf2
#        geometry_msgs
#        nav_msgs
#        tf2_geometry_msgs
#)

ament_auto_package(
        INSTALL_TO_SHARE
        launch
)

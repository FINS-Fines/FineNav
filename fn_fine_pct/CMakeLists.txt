cmake_minimum_required(VERSION 3.8)
project(fn_fine_pct)

set(CMAKE_CXX_STANDARD 17)

#TODO: 将架构调整为三个subdirectory
# 查找依赖
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_auto_find_build_dependencies()

# tomography 库
ament_auto_add_library(lib_tomography SHARED
        src/tomography/tomography.cpp
)
target_include_directories(lib_tomography PUBLIC include/tomography)
target_link_libraries(lib_tomography)

# A* 库
ament_auto_add_library(a_star SHARED
        src/planner/a_star.cpp
)
target_include_directories(a_star PUBLIC include/planner)

# 规划器
ament_auto_add_executable(fn_global_planner_node
        src/planner/pct_planner.cpp
)

target_include_directories(fn_global_planner_node PRIVATE include/planner)
target_link_libraries(fn_global_planner_node lib_tomography)

ament_auto_package(
        INSTALL_TO_SHARE
        rsc
)
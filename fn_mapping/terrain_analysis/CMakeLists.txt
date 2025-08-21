cmake_minimum_required(VERSION 3.22)
project(fn_terrain_analysis)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fn_map_interface REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/terrain_analysisi_base.cpp
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_find_gtest()
#    add_subdirectory(test)
endif ()

ament_auto_package()
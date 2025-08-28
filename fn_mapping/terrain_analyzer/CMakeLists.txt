cmake_minimum_required(VERSION 3.22)
project(fn_terrain_analyzer)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(fn_terrain_analysis_core REQUIRED)
find_package(fn_utils REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_simple SHARED
    src/simple_terrain_analyzer.cpp
)

pluginlib_export_plugin_description_file(fn_terrain_analysis_core simple_terrain_analyzer.xml)


if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_find_gtest()
#    add_subdirectory(test)
endif ()

ament_auto_package()
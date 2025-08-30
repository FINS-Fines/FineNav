cmake_minimum_required(VERSION 3.22)
project(fn_grid_map)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_find_gtest()
    add_subdirectory(test)
endif ()

ament_export_dependencies(
        eigen3_cmake_module
        Eigen3
)

ament_auto_package()
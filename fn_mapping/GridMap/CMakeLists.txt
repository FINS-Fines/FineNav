cmake_minimum_required(VERSION 3.22)
project(fn_grid_map)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

add_library(fn_grid_map INTERFACE)
target_include_directories(fn_grid_map INTERFACE
        include
        ${EIGEN3_INCLUDE_DIR}
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_find_gtest()
    add_subdirectory(test)
endif ()

ament_auto_package()
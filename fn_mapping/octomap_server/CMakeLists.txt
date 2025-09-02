cmake_minimum_required(VERSION 3.22)
project(fn_octomap_server)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(octomap REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/octomap_server.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC octomap)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_find_gtest()
endif ()

add_subdirectory(demo)

ament_auto_package()
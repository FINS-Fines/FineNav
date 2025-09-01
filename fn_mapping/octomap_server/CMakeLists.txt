cmake_minimum_required(VERSION 3.22)
project(fn_octomap_server)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

# 添加third_party中的octomap依赖
add_subdirectory(third_party/octomap-1.10.0)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)
include_directories(third_party/octomap-1.10.0/include)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/octomap_server.cpp
)

target_link_libraries(${PROJECT_NAME}  octomap)
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_find_gtest()
endif ()

ament_auto_package()
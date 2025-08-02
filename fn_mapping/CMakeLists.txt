cmake_minimum_required(VERSION 3.22)
project(fn_mapping)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
        include
)

ament_auto_add_executable(fn_mapping_node
        src/test.cpp
)
target_include_directories(fn_mapping_node PRIVATE ${EIGEN3_INCLUDE_DIR})

#ament_auto_add_library(fn_mapping SHARED
#        src/mapper.cpp
#        src/mapper_node.cpp
#        src/mapper_server.cpp
#)

ament_auto_package(
#        INSTALL_TO_SHARE
#        launch
#        config
#        rviz
#        maps
)
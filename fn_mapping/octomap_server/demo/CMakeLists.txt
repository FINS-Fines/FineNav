find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fn_utils REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

ament_auto_add_executable(${PROJECT_NAME}_demo
        octomap_server_demo.cpp
)
target_link_libraries(${PROJECT_NAME}_demo  ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_demo
        rclcpp
        sensor_msgs
        fn_utils
)

ament_auto_add_executable(pcd_to_octree
        pcd_to_octree.cpp
)
target_link_libraries(${PROJECT_NAME}_demo  ${PROJECT_NAME})
ament_target_dependencies(pcd_to_octree
    pcl_conversions
        pcl_ros
)

ament_auto_add_executable(octree_to_pcd
        octree_to_pcd.cpp
)
target_link_libraries(${PROJECT_NAME}_demo  ${PROJECT_NAME})
ament_target_dependencies(octree_to_pcd
        pcl_conversions
        pcl_ros
)

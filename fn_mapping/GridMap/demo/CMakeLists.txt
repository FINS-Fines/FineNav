find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(fn_utils REQUIRED)

ament_auto_add_executable(move_demo
        move_demo.cpp
)
target_include_directories(move_demo PRIVATE utils)
target_link_libraries(move_demo fn_grid_map)
ament_target_dependencies(move_demo
        rclcpp
        sensor_msgs
        visualization_msgs
        fn_utils
)

ament_auto_add_executable(raycast_demo
        raycast_demo.cpp
)
target_include_directories(raycast_demo PRIVATE utils)
target_link_libraries(raycast_demo fn_grid_map)
ament_target_dependencies(raycast_demo
        rclcpp
        sensor_msgs
        visualization_msgs
        fn_utils
)



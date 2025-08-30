find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(fn_utils REQUIRED)

ament_auto_add_executable(${PROJECT_NAME}_demo
        grid_map_demo.cpp
)
target_include_directories(${PROJECT_NAME}_demo PRIVATE utils)
target_link_libraries(${PROJECT_NAME}_demo fn_grid_map)
ament_target_dependencies(${PROJECT_NAME}_demo
        rclcpp
        sensor_msgs
        visualization_msgs
        fn_utils
)


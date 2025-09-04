find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fn_utils REQUIRED)

ament_auto_add_executable(${PROJECT_NAME}_demo
        octomap_server_demo.cpp
)

target_link_libraries(${PROJECT_NAME}_demo  ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}_demo
        rclcpp
        sensor_msgs
        fn_utils
)


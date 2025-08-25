cmake_minimum_required(VERSION 3.8)
project(fn_behavior)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/random_goal_node.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(fn_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(nav2_bt_navigator plugin.xml)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  pluginlib
        tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
)

ament_package()
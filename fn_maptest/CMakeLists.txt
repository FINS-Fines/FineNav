cmake_minimum_required(VERSION 3.8)
project(fn_maptest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# 包含头文件目录（指向include）
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 编译可执行文件
add_executable(pcd_publisher src/main.cpp)
ament_target_dependencies(pcd_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
)

# 链接PCL库
target_link_libraries(pcd_publisher
  ${PCL_LIBRARIES}
)

# 安装可执行文件
install(TARGETS
  pcd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（供其他包引用）
install(DIRECTORY include/
  DESTINATION include/
)

# 安装资源文件（如PCD文件）
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
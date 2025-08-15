cmake_minimum_required(VERSION 3.5)
project(fines_gazebo_simulator)

# 查找ROS 2基础依赖
find_package(ament_cmake REQUIRED)

# 安装launch文件到指定目录
install(DIRECTORY
  launch
  resource
  config
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# 声明包依赖（与package.xml对应）
ament_package()

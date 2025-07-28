cmake_minimum_required(VERSION 3.10)
project(ign_sim_pointcloud_tool)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 启用警告提示，强制更规范的代码
add_definitions(-Wall -Werror)

## 导出编译命令供clangd使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## 查找依赖项 ##
#######################

find_package(ament_cmake_auto REQUIRED)
# 自动查找依赖（包含原有依赖如rclcpp、pcl等，新增tf2相关依赖会在package.xml中声明）
ament_auto_find_build_dependencies()

###########
## 编译 ##
###########

# 编译共享库（包含两个节点的代码，避免重复编译）
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src  # 假设src目录下包含两个节点的实现（PointCloudConverter和TfCloudTransformer）
)

# 注册第一个节点：点云格式转换节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ign_sim_pointcloud_tool::PointCloudConverter  # 原有节点类
  EXECUTABLE point_cloud_converter_node  # 第一个节点的可执行文件名
)

# 注册第二个节点：TF坐标转换节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ign_sim_pointcloud_tool::TfCloudTransformer  # 新增节点类（需在代码中实现）
  EXECUTABLE tf_cloud_transformer_node  # 第二个节点的可执行文件名
)

#############
## 测试 ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  # 排除不需要的检查项
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## 安装 ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch  # 安装launch目录（如果有launch文件，需确保此处包含）
)
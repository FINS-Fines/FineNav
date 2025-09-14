#include "rclcpp/rclcpp.hpp"
#include "fn_maptest/pcd_publisher_node.hpp"  // 引用改名后的头文件

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  // 创建节点（使用改名后的类）
  auto node = std::make_shared<fn_maptest::PcdPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
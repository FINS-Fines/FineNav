#ifndef FN_MAPTEST_PCD_PUBLISHER_NODE_HPP_
#define FN_MAPTEST_PCD_PUBLISHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"
#include <string>
#include <filesystem>

namespace fs = std::filesystem;
namespace fn_maptest
{

class PcdPublisherNode : public rclcpp::Node
{
public:
  // 构造函数：通过ROS参数初始化
  PcdPublisherNode() : Node("pcd_publisher_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("pcd_file", "");
    this->declare_parameter<std::string>("topic_name", "output_pointcloud");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("publish_rate", 1.0);

    // 获取参数
    std::string pcd_file = this->get_parameter("pcd_file").as_string();
    std::string topic_name = this->get_parameter("topic_name").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // 检查PCD文件是否存在
    if (pcd_file.empty() || !fs::exists(pcd_file)) {
      RCLCPP_ERROR(this->get_logger(), "PCD文件不存在: %s", pcd_file.c_str());
      rclcpp::shutdown();
      return;
    }

    // 创建发布者
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);

    // 加载PCD文件
    if (!load_pcd_file(pcd_file)) {
      RCLCPP_ERROR(this->get_logger(), "加载PCD文件失败");
      rclcpp::shutdown();
      return;
    }

    // 创建定时器
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&PcdPublisherNode::publish_pointcloud, this)
    );

    RCLCPP_INFO(this->get_logger(), 
      "点云发布器初始化完成:\n"
      "  文件: %s\n"
      "  话题: %s\n"
      "  坐标系: %s\n"
      "  频率: %.2f Hz",
      pcd_file.c_str(), topic_name.c_str(), frame_id_.c_str(), publish_rate
    );
  }

private:
  // 加载PCD文件
  bool load_pcd_file(const std::string& file_path)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *temp_cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "无法加载PCD文件: %s", file_path.c_str());
      return false;
    }

    // 转换为ROS消息
    pcl::toROSMsg(*temp_cloud, cloud_msg_);
    RCLCPP_INFO(this->get_logger(), "成功加载PCD文件，包含 %zu 个点", temp_cloud->size());
    return true;
  }

  // 发布点云
  void publish_pointcloud()
  {
    cloud_msg_.header.stamp = this->now();
    cloud_msg_.header.frame_id = frame_id_;
    publisher_->publish(cloud_msg_);
  }

  // 成员变量
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  std::string frame_id_;
};

}  // namespace fn_maptest

#endif  // FN_MAPTEST_PCD_PUBLISHER_NODE_HPP_
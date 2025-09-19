#include "ign_sim_pointcloud_tool/cloud_frame_id_changer.hpp"

namespace ign_sim_pointcloud_tool
{
CloudFrameIdChanger::CloudFrameIdChanger(const rclcpp::NodeOptions & options)
: rclcpp::Node("cloud_frame_id_changer", options)
{
  // 声明并获取参数
  this->declare_parameter<std::string>("target_frame", "base_lidar");
  this->declare_parameter<std::string>("input_cloud_topic", "velodyne_points");
  this->declare_parameter<std::string>("output_cloud_topic", "modified_velodyne_points");

  target_frame_ = this->get_parameter("target_frame").as_string();
  input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
  output_cloud_topic_ = this->get_parameter("output_cloud_topic").as_string();

  // 创建订阅者和发布者
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&CloudFrameIdChanger::pointCloudCallback, this, std::placeholders::_1));

  modified_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_cloud_topic_, 
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
  );

  RCLCPP_INFO(
    this->get_logger(), "Cloud Frame ID Changer initialized. Target frame: %s, Input topic: %s, Output topic: %s",
    target_frame_.c_str(), input_cloud_topic_.c_str(), output_cloud_topic_.c_str());
}

void CloudFrameIdChanger::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  // 如果已经是目标坐标系，直接发布
  if (cloud_msg->header.frame_id == target_frame_) {
    modified_cloud_pub_->publish(*cloud_msg);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Frame ID already matches target, published without changes. Frame: %s",
      target_frame_.c_str()
    );
    return;
  }

  // 创建新的消息副本，仅修改frame_id
  sensor_msgs::msg::PointCloud2 modified_msg = *cloud_msg;
  modified_msg.header.frame_id = target_frame_;
  
  // 发布修改后的点云
  modified_cloud_pub_->publish(modified_msg);
  
  RCLCPP_DEBUG(
    this->get_logger(),
    "Modified point cloud frame ID from %s to %s. Number of points: %u",
    cloud_msg->header.frame_id.c_str(),
    target_frame_.c_str(),
    cloud_msg->width * cloud_msg->height
  );
}

}  // namespace ign_sim_pointcloud_tool

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ign_sim_pointcloud_tool::CloudFrameIdChanger)

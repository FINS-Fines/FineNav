#ifndef CLOUD_FRAME_ID_CHANGER_HPP
#define CLOUD_FRAME_ID_CHANGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ign_sim_pointcloud_tool
{
class CloudFrameIdChanger : public rclcpp::Node
{
public:
  explicit CloudFrameIdChanger(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
  
  // 订阅者和发布者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr modified_cloud_pub_;
  
  // 参数
  std::string target_frame_;
  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
};

}  // namespace ign_sim_pointcloud_tool

#endif  // CLOUD_FRAME_ID_CHANGER_HPP

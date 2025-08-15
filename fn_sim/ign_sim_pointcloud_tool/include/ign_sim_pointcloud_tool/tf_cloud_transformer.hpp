#ifndef IGN_SIM_POINTCLOUD_TOOL__TF_CLOUD_TRANSFORMER_HPP_
#define IGN_SIM_POINTCLOUD_TOOL__TF_CLOUD_TRANSFORMER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "Eigen/Geometry"

namespace ign_sim_pointcloud_tool
{
class TfCloudTransformer : public rclcpp::Node
{
public:
  explicit TfCloudTransformer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
  
  // TF缓冲和监听器
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // 订阅者和发布者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
  
  // 参数
  std::string target_frame_;
  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  double tf_timeout_;
  std::string base_lidar;
};
}  // namespace ign_sim_pointcloud_tool

#endif  // IGN_SIM_POINTCLOUD_TOOL__TF_CLOUD_TRANSFORMER_HPP_

#include "ign_sim_pointcloud_tool/tf_cloud_transformer.hpp"
#include <pcl/registration/transforms.h>

namespace ign_sim_pointcloud_tool
{
TfCloudTransformer::TfCloudTransformer(const rclcpp::NodeOptions & options)
: rclcpp::Node("tf_cloud_transformer", options)
{
  // 声明并获取参数
  this->declare_parameter<std::string>("target_frame", "base_link");
  this->declare_parameter<std::string>("input_cloud_topic", "velodyne_points");
  this->declare_parameter<std::string>("output_cloud_topic", "transformed_velodyne_points");
  this->declare_parameter<double>("tf_timeout", 0.1);
  this->declare_parameter<std::string>("base_lidar", "base_lidar");

  target_frame_ = this->get_parameter("target_frame").as_string();
  input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
  output_cloud_topic_ = this->get_parameter("output_cloud_topic").as_string();
  tf_timeout_ = this->get_parameter("tf_timeout").as_double();
  base_lidar = this->get_parameter("base_lidar").as_string();

  // 初始化TF缓冲和监听器
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建订阅者和发布者
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TfCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

  transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
  output_cloud_topic_, 
  rclcpp::QoS(rclcpp::KeepLast(10)).reliable()  // 改为可靠传输
);

  RCLCPP_INFO(
    this->get_logger(), "TF Cloud Transformer initialized. Target frame: %s, Input topic: %s",
    target_frame_.c_str(), input_cloud_topic_.c_str());
}

void TfCloudTransformer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  // 如果已经是目标坐标系，直接发布
  if (base_lidar == target_frame_) {
    transformed_cloud_pub_->publish(*cloud_msg);
    return;
  }

  try {
    // 获取TF变换
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      target_frame_,
      base_lidar,
      cloud_msg->header.stamp,
      tf2::durationFromSec(tf_timeout_)
    );

    // 将点云转换为PCL格式进行处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 应用坐标变换
    Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, eigen_transform);

    // 转换回ROS消息并发布
    sensor_msgs::msg::PointCloud2 transformed_msg;
    pcl::toROSMsg(*transformed_cloud, transformed_msg);
    transformed_msg.header.frame_id = target_frame_;
    transformed_msg.header.stamp = cloud_msg->header.stamp;
    
    transformed_cloud_pub_->publish(transformed_msg);
    
    RCLCPP_DEBUG(
      this->get_logger(),
      "Transformed point cloud from %s to %s. Number of points: %zu",
      base_lidar.c_str(),
      target_frame_.c_str(),
      cloud->size()
    );
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not transform point cloud from %s to %s: %s",
      base_lidar.c_str(),
      target_frame_.c_str(),
      ex.what()
    );
  }
}

}  // namespace ign_sim_pointcloud_tool

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ign_sim_pointcloud_tool::TfCloudTransformer)

// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_LOCALIZATION_MANAGER_H
#define FINENAV2D_LOCALIZATION_MANAGER_H

#include <string>

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace finenav_2d {

class LocalizationManager : public rclcpp::Node {
  public:
    enum SourceType { kLocal, kGlobal, kGlobalWhenInitialize };
    enum MessageTye { kOdometry, kPoseWithCovarianceStamped };

    explicit LocalizationManager(const rclcpp::NodeOptions& options);

  private:
    /**
    * @brief nav_msgs::msg::Odometry 消息回调
    */
    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const;

    /**
    * @brief geometry_msgs::msg::PoseWithCovarianceStamped 消息回调
    */
    void PoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const;

    // 用于处理不同消息类型的定位数据
    rclcpp::SubscriptionBase::SharedPtr sub;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string parent_frame_;
    std::string child_frame_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS2 Parameters
    int source_type_;
    int message_tye_;
    bool enable_;

    std::string topic_;

    std::string map_frame_;
    std::string odom_frame_;
    std::string base_link_frame_;
};

}  // namespace finenav_2d

#endif  //FINENAV2D_LOCALIZATION_MANAGER_H

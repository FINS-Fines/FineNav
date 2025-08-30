// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 25-8-22.
//

#ifndef ODOMETRY_GZ_MANAGER_H
#define ODOMETRY_GZ_MANAGER_H


#include <string>

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"


namespace finenav_2d {

class Odometry_gz_manager : public rclcpp::Node {
public:
    explicit Odometry_gz_manager(const rclcpp::NodeOptions& options);
private:


    /**
    * @brief geometry_msgs::msg::PoseWithCovarianceStamped 消息回调
    */
    void PoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) ;

    // 用于处理不同消息类型的定位数据
    rclcpp::SubscriptionBase::SharedPtr sub;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;



};

}  // namespace finenav_2d

#endif  //FINENAV2D_LOCALIZATION_MANAGER_H





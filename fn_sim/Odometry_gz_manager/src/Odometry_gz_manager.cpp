// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 25-8-22.
//
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "Odometry_gz_manager.h"

namespace finenav_2d {

Odometry_gz_manager::Odometry_gz_manager(const rclcpp::NodeOptions& options)
    : rclcpp::Node(options.arguments()[0], options) {


    // 初始化 TF Buffer 和 TransformListener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/world/default/dynamic_pose/info", 10, std::bind(&Odometry_gz_manager::PoseCallback, this, std::placeholders::_1));

}


void Odometry_gz_manager::PoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)  {
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty PoseArray, skipping.");
        return;
    }

    const auto& pose_current = msg->poses[0];
    tf2::Transform T_init, T_cur;
    T_init.setOrigin(tf2::Vector3(-2.0, -0.5, 0.8));
    T_init.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    T_cur.setOrigin(tf2::Vector3(
        pose_current.position.x,
        pose_current.position.y,
        pose_current.position.z
    ));
    T_cur.setRotation(tf2::Quaternion(
        pose_current.orientation.x,
        pose_current.orientation.y,
        pose_current.orientation.z,
        pose_current.orientation.w
    ));

    // 相对初始位姿的变换
    tf2::Transform T_rel = T_init.inverseTimes(T_cur);

    // 转成 TransformStamped
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = this->get_clock()->now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";

    odom_tf.transform.translation.x = T_rel.getOrigin().x();
    odom_tf.transform.translation.y = T_rel.getOrigin().y();
    odom_tf.transform.translation.z = T_rel.getOrigin().z();
    odom_tf.transform.rotation = tf2::toMsg(T_rel.getRotation());

    // 发布 TF
    tf_broadcaster_->sendTransform(odom_tf);
}




} // namespace finenav_2d
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "localization_manager.h"

namespace finenav_2d {

LocalizationManager::LocalizationManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node(options.arguments()[0], options) {

    // Init Parameters
    source_type_ = this->declare_parameter("source_type", 0);
    enable_ = this->declare_parameter("enable", true);
    topic_ = this->declare_parameter("topic", "/localization");
    message_tye_ = this->declare_parameter("message_type", 0);
    map_frame_ = this->declare_parameter("map_frame", "map");
    odom_frame_ = this->declare_parameter("odom_frame", "odom");
    base_link_frame_ = this->declare_parameter("base_link_frame", "base_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // 处理坐标系
    switch (source_type_) {
        case kLocal:
            parent_frame_ = odom_frame_;
            child_frame_ = base_link_frame_;
            break;
        case kGlobal: case kGlobalWhenInitialize:
            parent_frame_ = map_frame_;
            child_frame_ = odom_frame_;
            break;
    }

    // 如果没有启用，为了保证 map_frame_->odom_frame_->base_link_frame_ 树完整，需要发布静态同等变换
    if (!enable_) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = parent_frame_;
        transform.child_frame_id = child_frame_;
        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;
        transform.transform.rotation.x = 0;
        transform.transform.rotation.y = 0;
        transform.transform.rotation.z = 0;
        transform.transform.rotation.w = 1;
        tf_static_broadcaster_->sendTransform(transform);
    } else {
        if( message_tye_ == kOdometry) {
            sub = this->create_subscription<nav_msgs::msg::Odometry>(
                topic_, 10, std::bind(&LocalizationManager::OdometryCallback, this, std::placeholders::_1));
        } else if (message_tye_ == kPoseWithCovarianceStamped) {
            sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                topic_, 10, std::bind(&LocalizationManager::PoseCallback, this, std::placeholders::_1));
        }
    }
}

void LocalizationManager::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const {

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;

    // 处理不同的 source_type_
    if (source_type_ == kLocal) {
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_link_frame_;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);

    } else if (source_type_ == kGlobalWhenInitialize) {
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = odom_frame_;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;
        tf_static_broadcaster_->sendTransform(transform);

    } else if (source_type_ == kGlobal) {
        // 从消息中获取 map 到 base_link 的变换
        geometry_msgs::msg::TransformStamped map_to_baselink;
        map_to_baselink.transform.translation.x = msg->pose.pose.position.x;
        map_to_baselink.transform.translation.y = msg->pose.pose.position.y;
        map_to_baselink.transform.translation.z = msg->pose.pose.position.z;
        map_to_baselink.transform.rotation = msg->pose.pose.orientation;

        // 从tf树中获取从 odom 到 base_link 的变换
        geometry_msgs::msg::TransformStamped odom_to_base_link;
        try {
            odom_to_base_link = tf_buffer_->lookupTransform(odom_frame_, base_link_frame_, tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
            return;
        }

        // 解算 map 到 odom 的变换，转换为 tf2::transform 以便于计算
        tf2::Transform tf_map_to_base_link;
        tf2::Transform tf_odom_to_base_link;
        tf2::Transform tf_map_to_odom;
        tf2::fromMsg(map_to_baselink.transform, tf_map_to_base_link);
        tf2::fromMsg(odom_to_base_link.transform, tf_odom_to_base_link);
        tf_map_to_odom.mult(tf_map_to_base_link, tf_odom_to_base_link.inverse());

        // 发布 map 到 odom 的变换
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = odom_frame_;
        transform.transform = tf2::toMsg(tf_map_to_odom);
        tf_broadcaster_->sendTransform(transform);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown source type: %d", source_type_);
    }
}

void LocalizationManager::PoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;

    // 声明共享的变量，确保所有变量都在 switch 语句之前声明
    geometry_msgs::msg::TransformStamped odom_to_base_link;
    geometry_msgs::msg::TransformStamped map_to_baselink;
    tf2::Transform map_to_base_link_transform;
    tf2::Transform odom_to_base_link_transform;
    tf2::Transform odom_to_base_link_inverse;
    tf2::Quaternion combinedRotation;
    tf2::Vector3 combinedTranslation;
    tf2::Transform map_to_odom_transform;

    // 处理不同的 source_type_
    switch (source_type_) {
        case kLocal:
            transform.header.frame_id = odom_frame_;
            transform.child_frame_id = base_link_frame_;
            transform.transform.translation.x = msg->pose.pose.position.x;
            transform.transform.translation.y = msg->pose.pose.position.y;
            transform.transform.translation.z = msg->pose.pose.position.z;
            transform.transform.rotation = msg->pose.pose.orientation;
            tf_broadcaster_->sendTransform(transform);
            break;

        case kGlobalWhenInitialize:
            transform.header.frame_id = map_frame_;
            transform.child_frame_id = odom_frame_;
            transform.transform.translation.x = msg->pose.pose.position.x;
            transform.transform.translation.y = msg->pose.pose.position.y;
            transform.transform.translation.z = msg->pose.pose.position.z;
            transform.transform.rotation = msg->pose.pose.orientation;
            tf_static_broadcaster_->sendTransform(transform);
            break;

        case kGlobal:
            transform.header.frame_id = map_frame_;
            transform.child_frame_id = odom_frame_;

            // 尝试获取从 odom 到 base_link 的变换
            try {
                odom_to_base_link = tf_buffer_->lookupTransform(odom_frame_, base_link_frame_, tf2::TimePointZero);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
                return;
            }

            map_to_baselink.transform.translation.x = msg->pose.pose.position.x;
            map_to_baselink.transform.translation.y = msg->pose.pose.position.y;
            map_to_baselink.transform.translation.z = msg->pose.pose.position.z;
            map_to_baselink.transform.rotation = msg->pose.pose.orientation;

            // 将 geometry_msgs 转换为 tf2 对象
            tf2::fromMsg(map_to_baselink.transform, map_to_base_link_transform);
            tf2::fromMsg(odom_to_base_link.transform, odom_to_base_link_transform);

            // 计算合成变换：map_to_odom_transform = map_to_base_link_transform * odom_to_base_link_inverse
            odom_to_base_link_inverse = odom_to_base_link_transform.inverse();

            combinedRotation = map_to_base_link_transform.getRotation() * odom_to_base_link_inverse.getRotation();
            combinedTranslation =
                quatRotate(map_to_base_link_transform.getRotation(), odom_to_base_link_inverse.getOrigin()) +
                map_to_base_link_transform.getOrigin();

            // 设置合成后的变换
            map_to_odom_transform.setRotation(combinedRotation);
            map_to_odom_transform.setOrigin(combinedTranslation);

            // 将合成后的变换转换回 geometry_msgs::TransformStamped
            transform.transform = tf2::toMsg(map_to_odom_transform);
            tf_broadcaster_->sendTransform(transform);
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown source type: %d", source_type_);
            return;  // 在 default 处理完后直接返回，避免继续执行
    }
}

} // namespace finenav_2d
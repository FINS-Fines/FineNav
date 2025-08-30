// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace finenav_utils {

class CloudPublishHelper {
public:
    CloudPublishHelper() = default;
    ~CloudPublishHelper() = default;

    void configure(const std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>& pub,
               bool is_dense, const std::string& frame_id);

    void addPoint(const float& x, const float& y, const float& z,
                  const std::array<uint8_t, 3>& rgb = {255, 255, 255});

    // 需要传入rclcpp的时间
    void publish(rclcpp::Time time);

private:
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
};

} // namespace finenav_utils

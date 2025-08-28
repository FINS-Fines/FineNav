// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "cloud_publish_helper.hpp"

namespace finenav_utils {

void CloudPublishHelper::configure(const std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>& pub,
                    bool is_dense, const std::string& frame_id) {
    pub_ = pub;
    cloud.is_dense = is_dense;
    cloud.header.frame_id = frame_id;
};

void CloudPublishHelper::addPoint(const float& x, const float& y, const float& z,
                  const std::array<uint8_t, 3>& rgb) {
    pcl::PointXYZRGB point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.r = rgb[0];
    point.g = rgb[1];
    point.b = rgb[2];
    cloud.points.push_back(point);
}


void CloudPublishHelper::publish(rclcpp::Time time) {

    cloud.width = cloud.points.size();
    cloud.height = 1;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = time;
    pub_->publish(cloud_msg);

    cloud.clear();
}



} // namespace finenav_utils


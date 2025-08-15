// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "map_manager.h"

namespace finenav_2d {

MapManager::MapManager(const rclcpp::NodeOptions& options)
    : Node(options.arguments()[0], options), local_map_({20.0, 20.0, 20.0}, 0.5) {

    RCLCPP_INFO(get_logger(), "MapManager initialized");


}

void MapManager::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // 改写成message_filters实现的tf接收，文档......
    // 参考octomap_server

    // 中间流程
    // 算出来world_frame下base_link在哪儿
    // moveTo()








    // 发布地图（可视化）
    publishLocalMap();


}

void MapManager::publishLocalMap() {
    // 点云形式发出来
}




}

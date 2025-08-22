// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "map_manager.h"

#include "terrain_analysis_base.hpp"

#include <pluginlib/class_loader.hpp>

namespace finenav_2d {

MapManager::MapManager(const rclcpp::NodeOptions& options)
    : Node(options.arguments()[0], options), local_map_({20.0, 20.0, 20.0}, 0.5) {


    pluginlib::ClassLoader<TerrainAnalyzerBase> tta_loader("fn_terrain_analysis_core", "finenav_2d::TerrainAnalyzerBase");

    try {
        auto terrain_analyzer = tta_loader.createSharedInstance("fn_terrain_analysis_plugins::SimpleTerrainAnalyzer");
    }
    catch(pluginlib::PluginlibException& ex) {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }

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

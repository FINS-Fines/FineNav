// Copyright (c) 2025. 
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include "terrain_analysis_simple.hpp"


namespace finenav_2d {

void SimpleTerrainAnalyzer::analyzeTerrain() {
    // 获取地图的最小和最大索引
    auto min_idx = map_interface_->getMinIndex();
    auto max_idx = map_interface_->getMaxIndex();

    // 创建terrain_test属性字段
    map_interface_->getAttributeFields().addAttributeField("terrain_test", min_idx, max_idx, NAN);

    // 遍历所有索引
    for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
        for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
            for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                Index current_idx(x, y, z);

                // 检查当前索引是否被占用
                if (map_interface_->isOccupied(current_idx)) {
                    map_interface_->getAttributeFields().at("terrain_test", current_idx) = 1.0f;
                }
            }
        }
    }
}



} // namespace finenav_2d

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(finenav_2d::SimpleTerrainAnalyzer, finenav_2d::TerrainAnalyzerBase)

// Copyright (c) 2025. 
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include "simple_terrain_analyzer.hpp"


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


    /****** 上面的代码只是一个示例，实际的地形分析逻辑会更复杂 ******/

    // addAttributeField 得设置成二维的大小，很简单

    // 分析：遍历地图数据，针对每个(x,y)，可以获得该位置上的一串z

    // 从下往上遍历，找到占据点。【为了测试起见，先拿到第一个占据点，然后发布出来，检测框架正确性】

    // 把每个(x,y)能走或不能走，放到attribute field里
}



} // namespace finenav_2d

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(finenav_2d::SimpleTerrainAnalyzer, finenav_2d::TerrainAnalyzerBase)

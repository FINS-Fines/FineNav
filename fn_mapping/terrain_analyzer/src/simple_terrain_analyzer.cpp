// Copyright (c) 2025. 
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include "simple_terrain_analyzer.hpp"

namespace finenav_2d {

void SimpleTerrainAnalyzer::analyzeTerrain() {

    // 创建terrain_test属性字段
    // terrain_data_view_->getAttributeFields().addAttributeField("terrain_test", attr_min_idx, attr_max_idx, NAN);

    // 遍历所有索引
    // 遍历二维平面 X-Y

    for (size_t x = 0; x < interface_->sizeX(); ++x) {
        for (size_t y = 0; y < interface_->sizeY() ; ++y) {

            auto z_values = interface_->getMap(x, y);
            // 这里可以对 z_values 进行分析，例如计算最大值、最小值、平均值等
            if (!z_values.empty()) {
                double max_z = *std::max_element(z_values.begin(), z_values.end());
                double min_z = *std::min_element(z_values.begin(), z_values.end());
                double avg_z = std::accumulate(z_values.begin(), z_values.end(), 0.0) / z_values.size();

                // 将分析结果存储到属性字段中
                // terrain_data_view_->getAttributeFields().at("terrain_test", Index(x, y, 0)) = static_cast<float>(avg_z);
            } else {
                // 如果没有数据，可以设置为 NaN 或其他默认值
                // terrain_data_view_->getAttributeFields().at("terrain_test", Index(x, y, 0)) = NAN;
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

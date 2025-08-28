// Copyright (c) 2025. 
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <ranges>

#include "simple_terrain_analyzer.hpp"



namespace finenav_2d {


void SimpleTerrainAnalyzer::configure(
    const rclcpp::Node::WeakPtr & parent,
    std::string name,
    const TerrainAnalyzerInterface::Ptr &map_interface) {

    node_ = parent;
    interface_ = map_interface;
    name_ = name;

    auto node = parent.lock();
    node->declare_parameter(name + ".example_param", 1.0);

    // TODO: 输入参数，决定发布字段

    pub_ground_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_analyzer/debug_cloud", 10);

} // namespace finenav_2d

void SimpleTerrainAnalyzer::analyzeTerrain() {

    /*********** 发布示例 *************/






    /*********** 发布示例 *************/


    /******** Example ************/
    for (size_t x = 0; x < interface_->sizeX(); ++x) {
        for (size_t y = 0; y < interface_->sizeY() ; ++y) {

            auto z_values = interface_->getMap(x, y);
            auto filtered_z_values = z_values | std::views::filter([this](const float& value) {
                return interface_->dataIsValid(value);
            });

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
    /******** Example ************/
    // TODO: ground和ceiling不存在值 1） NAN 2） std::neumet
    // std::numeric_limits<float>::max(); // 最大值
    // std::numeric_limits<float>::min(); // 最小正值
    // std::numeric_limits<float>::lowest(); // 最小值

    const float OCCUPIED_VALUE = 100.0f;      // 全占据特殊值
    const float FREE_VALUE = -100.0f;         // 全空闲特殊值
    const float MAX_GRADIENT = 1.0f;            // 最大允许坡度（梯度阈值）
    const float ROBOT_HEIGHT = 0.6f;            // 机器人最小通过高度


    // 遍历所有索引
    // 遍历二维平面 X-Y
    for (size_t x = 0; x < interface_->sizeX(); ++x) {
        for (size_t y = 0; y < interface_->sizeY() ; ++y) {

            // TODO: 过会儿大改
            std::vector<int> ground_indices;
            std::vector<int> ceiling_indices;
            // 沿 Z 方向从下到上扫描 处理ground和ceiling信息
            for (int z = min_idx.z(); z < max_idx.z(); ++z) {
                Index idx(x, y, z);
                Index idx_next(x, y, z+1);
                if (interface_->isOccupied(idx) && !interface_->isOccupied(idx_next)){
                        ground_indices.push_back(z);
                }
                if (!interface_->isOccupied(idx)  && ceiling_indices.size()<ground_indices.size() && interface_->isOccupied(idx_next)) {
                        ceiling_indices.push_back(z);
                }
                // TODO： Ensure出来的时候保证ground和ceiling一一对应

            }
            // if(ceiling_indices.size()<ground_indices.size()){
            //     ceiling_indices.push_back(max_idx.z()+1); // 添加一个虚拟的顶点，表示地图顶部
            // }
            //
            // if(ground_indices.empty()){
            //     if(interface_->isOccupied({x,y,0})){
            //         interface_->getAttributeFields().at("Ground", terrain_idx) = OCCUPIED_VALUE;
            //         interface_->getAttributeFields().at("Ceiling", terrain_idx) = OCCUPIED_VALUE;
            //         interface_->getAttributeFields().at("Height", terrain_idx) = OCCUPIED_VALUE;
            //     }else{
            //         interface_->getAttributeFields().at("Ground", terrain_idx) = FREE_VALUE;
            //         interface_->getAttributeFields().at("Ceiling", terrain_idx) = FREE_VALUE;
            //         interface_->getAttributeFields().at("Height", terrain_idx) = FREE_VALUE;
            // }
            // }
            // else{
            //     int effective_index =0;
            //     for (int i=1 ; i< ground_indices.size();++i){
            //         if(fabs(ground_indices[i]) < fabs(ground_indices[effective_index]) && ((ceiling_indices[i]-ground_indices[i])>2)){
            //             effective_index = i;
            //         }
            //     }
            //     interface_->getAttributeFields().at("Ground", terrain_idx) = ground_indices[effective_index];
            //     interface_->getAttributeFields().at("Ceiling", terrain_idx) = ceiling_indices[effective_index];
            //     interface_->getAttributeFields().at("Height", terrain_idx) = (ceiling_indices[effective_index]-ground_indices[effective_index])*interface_->getResolution();
            //
            // }

            // 可视化ground
            pub_helper_.configure(pub_ground_, true, "map");
            pub_helper_.addPoint(1, 1, 1, {255, 0, 0});

            pub_helper_.publish(node_.lock()->now());

        }
    }


    for (size_t x = 0; x < interface_->sizeX(); ++x) {
        for (size_t y = 0; y < interface_->sizeY() ; ++y) {
            // 定义四个方向：上、右、下、左
            int dx[4] = {0, 1, 0, -1};
            int dy[4] = {1, 0, -1, 0};

            float terrain_value = 0;

            // 检查当前格子本身
            if (current_ground == OCCUPIED_VALUE) {
                // 全占据，不可通行
                terrain_value = 1;
            } else if (current_ground == FREE_VALUE) {
                // 全空闲，需要检查高度（可能是悬崖或深坑）
                terrain_value = 0; // 默认不可通行，或者根据需求调整
            } else {
                // 检查高度是否足够
                if (current_height < ROBOT_HEIGHT) {
                    terrain_value = 1;
                } else {
                    // 检查与周围四个格子的梯度
                    for (int i = 0; i < 4; ++i) {
                        int nx = x + dx[i];
                        int ny = y + dy[i];

                        // 检查是否越界
                        if (nx < min_idx.x() || nx > max_idx.x() ||
                            ny < min_idx.y() || ny > max_idx.y()) {
                            continue; // 忽略边界格子
                        }

                        Index neighbor_idx(nx, ny, 0);
                        float neighbor_ground = interface_->getAttributeFields().at("Ground", neighbor_idx);

                        // 处理特殊值
                        if (neighbor_ground == OCCUPIED_VALUE) {
                            // 邻居全占据，梯度过大
                            terrain_value = 1;
                            break;
                        } else if (neighbor_ground == FREE_VALUE) {
                            // 邻居全空闲，梯度过大（可能是悬崖）
                            continue;
                        } else {
                            // 计算梯度（高度差）
                            float gradient =fabs(interface_->getZheightat({x,y,current_ground}) -interface_->getZheightat({x,y,neighbor_ground})) /interface_->getResolution();
                            if (gradient>interface_->getAttributeFields().at("gradient", idx)){
                                interface_->getAttributeFields().at("gradient", idx) = gradient;
                            }
                            if (gradient > MAX_GRADIENT) {
                                terrain_value = 1;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

}



} // namespace finenav_2d

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(finenav_2d::SimpleTerrainAnalyzer, finenav_2d::TerrainAnalyzerBase)

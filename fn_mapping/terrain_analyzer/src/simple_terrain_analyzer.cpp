// Copyright (c) 2025. 
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <ranges>
#include <Eigen/Core>

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

    const float MAX_GRADIENT = 1.0f;            // 最大允许坡度（梯度阈值）
    const float ROBOT_HEIGHT = 0.6f;            // 机器人最小通过高度

    size_t size_x = interface_->sizeX();
    size_t size_y = interface_->sizeY();

    Eigen::ArrayXXf ground_array = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);
    Eigen::ArrayXXf ceiling_array = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);
    /******** Example ************/
    for (size_t x = 0; x < size_x; ++x) {
        for (size_t y = 0; y < size_y ; ++y) {

            auto z_values_before = interface_->getMap(x, y);
            auto z_values = z_values_before | std::views::filter([&](const float& z) {
                return interface_->dataIsValid(z);
            });

            float nearest_ground = std::numeric_limits<float>::max(); // 距离当前位置最近的ground


            if(!z_values.empty()) {
                for (auto it = z_values.begin(); it != z_values.end(); ++it) {
                    if (*std::next(it) - *it > ROBOT_HEIGHT) {  //筛选机器人可通过的ground和ceiling
                        if(fabs(*it) < fabs(nearest_ground)){  //筛选距离机器人最近的ground和ceiling
                            nearest_ground = *it;
                        }
                    }
           		}

                if (fabs(z_values.back()) < fabs(nearest_ground)){   //针对最上方的ground（没有ceiling）做一个判断
                    nearest_ground = z_values.back();
                    }
            } else {
                nearest_ground = NAN;
            }
            ground_array(x, y) = nearest_ground;   // 记录ground NAN代表全空 其他值为有效值
        }
    }



    // 可视化ground
    pub_helper_.configure(pub_ground_, true, "map");
    pub_helper_.addPoint(1, 1, 1, {255, 0, 0});
	pub_helper_.publish(node_.lock()->now());


    // 计算高度差以推断可通行性
    for (size_t x = 0; x < size_x; ++x) {
        for (size_t y = 0; y < size_y ; ++y) {
            // 定义四个方向：上、右、下、左
            int dx[4] = {0, 1, 0, -1};
            int dy[4] = {1, 0, -1, 0};

            float terrain_value = 0;  // 默认可通行
			float current_ground = ground_array(x, y);  //读取当前格子的ground高度

            // 检查当前格子本身
            if (!std::isnan(current_ground)) {   // 当前格子有ground
                 // 检查与周围四个格子的梯度
                for (int i = 0; i < 4; ++i) {
                    int nx = x + dx[i];
                    int ny = y + dy[i];

                    // 检查是否越界
                    if (nx < 0 || nx > size_x ||
                        ny < 0 || ny > size_y) {
                        continue; // 忽略边界格子
                        }

                    float neighbor_ground = ground_array(nx, ny);    // 读取邻居格子的ground高度

                    // 处理特殊值
					if (!std::isnan(neighbor_ground)) {   // 邻居格子有ground
                        // 计算高度差
                        float gradient = fabs(current_ground - neighbor_ground);
                        if (gradient > MAX_GRADIENT) {
                            terrain_value = 1;
                            break;
                        }
                    }
                }
            }
            interface_->setResult(x, y, terrain_value);
        }
    }

}



} // namespace finenav_2d

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(finenav_2d::SimpleTerrainAnalyzer, finenav_2d::TerrainAnalyzerBase)

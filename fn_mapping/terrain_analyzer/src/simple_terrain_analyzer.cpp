// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <ranges>
#include <Eigen/Core>

#include "simple_terrain_analyzer.hpp"

namespace finenav_2d {

const float MAX_GRADIENT = 0.06f;  // 最大允许坡度（梯度阈值）
const float ROBOT_HEIGHT = 0.4f;   // 机器人最小通过高度
const int KERNEL_SIZE = 3;      // 中值滤波核大小

void SimpleTerrainAnalyzer::configure(const rclcpp::Node::WeakPtr& parent, std::string name,
                                      const TerrainAnalyzerInterface::Ptr& map_interface) {

    node_ = parent;
    interface_ = map_interface;
    name_ = name;

    auto node = parent.lock();
    node->declare_parameter(name + ".example_param", 1.0);

    // TODO: 输入参数，决定发布字段
}  // namespace finenav_2d

void SimpleTerrainAnalyzer::analyzeTerrain(const float robot_pose_z) {

    size_t size_x = interface_->sizeX();
    size_t size_y = interface_->sizeY();

    Eigen::ArrayXXf ground_array = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);
    Eigen::ArrayXXf ceiling_array = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);  // TODO: 应该删掉

    /******** Example ************/
    for (size_t x = 0; x < size_x; ++x) {
        for (size_t y = 0; y < size_y; ++y) {

            auto z_values_before = interface_->getMap(x, y);
            auto z_values =
                z_values_before | std::views::filter([&](const float& z) { return interface_->dataIsValid(z); });

            float nearest_ground = std::numeric_limits<float>::max();  // 距离当前位置最近的ground

            if (!z_values.empty()) {
                for (auto it = z_values.begin(); it != z_values.end(); ++it) {
                    if (*std::next(it) - *it > ROBOT_HEIGHT) {  //筛选机器人可通过的ground和ceiling
                        if (fabs(*it - robot_pose_z) <
                            fabs(nearest_ground - robot_pose_z)) {  //筛选距离机器人最近的ground和ceiling
                            nearest_ground = *it;
                        }
                    }
                }

                if (fabs(z_values.back() - robot_pose_z) <
                    fabs(nearest_ground - robot_pose_z)) {  //针对最上方的ground（没有ceiling）做一个判断
                    nearest_ground = z_values.back();
                }
            } else {
                nearest_ground = NAN;
            }
            ground_array(x, y) = nearest_ground;  // 记录ground NAN代表全空 其他值为有效值
            interface_->setResult("valid_ground", x, y, nearest_ground);
        }
    }

    // TODO：对ground_array(x, y)进行中值滤波
    ground_array = median_filtering(ground_array, KERNEL_SIZE);

    // 计算高度差以推断可通行性
    for (size_t x = 0; x < size_x; ++x) {
        for (size_t y = 0; y < size_y; ++y) {
            // 定义四个方向：上、右、下、左
            int dx[4] = {0, 1, 0, -1};
            int dy[4] = {1, 0, -1, 0};

            float terrain_value = 0;                    // 默认可通行
            float current_ground = ground_array(x, y);  //读取当前格子的ground高度
            // 检查当前格子本身
            if (!std::isnan(current_ground)) {  // 当前格子有ground
                                                // 检查与周围四个格子的梯度
                for (int i = 0; i < 4; ++i) {
                    int nx = x + dx[i];
                    int ny = y + dy[i];

                    // 检查是否越界
                    if (nx < 0 || nx > size_x - 1 || ny < 0 || ny > size_y - 1) {
                        continue;  // 忽略边界格子
                    }

                    float neighbor_ground = ground_array(nx, ny);  // 读取邻居格子的ground高度
                                                                   // 处理特殊值
                    if (!std::isnan(neighbor_ground)) {            // 邻居格子有ground
                        // 计算高度差
                        float gradient = fabs(current_ground - neighbor_ground);
                        if (gradient > MAX_GRADIENT) {
                            terrain_value = 1;
                            break;
                        }
                    }
                }
            }
            interface_->setResult(
                "traversablilty", x, y,
                terrain_value);  // TODO: 由用户输入一个string的SET进来，检查该字段是否在SET中，决定是否发布该字段数据
        }
    }
}



Eigen::ArrayXXf SimpleTerrainAnalyzer::median_filtering (const Eigen::ArrayXXf& input_array, const int kernel_size = 3) {
    int rows = input_array.rows();
    int cols = input_array.cols();
    Eigen::ArrayXXf output_array = Eigen::ArrayXXf::Constant(rows, cols, NAN);

    int k = kernel_size / 2;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            std::vector<float> neighbors;

            for (int m = -k; m <= k; ++m) {
                for (int n = -k; n <= k; ++n) {
                    int ni = i + m;
                    int nj = j + n;

                    if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
                        float val = input_array(ni, nj);
                        if (!std::isnan(val)) {
                            neighbors.push_back(val);
                        }
                    }
                }
            }

            if (!neighbors.empty()) {
                std::sort(neighbors.begin(), neighbors.end());
                size_t mid = neighbors.size() / 2;
                if (neighbors.size() % 2 == 0) {
                    output_array(i, j) = (neighbors[mid - 1] + neighbors[mid]) / 2.0f;
                } else {
                    output_array(i, j) = neighbors[mid];
                }
            }
        }
    }
    
    return output_array;
}
}  // namespace finenav_2d



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(finenav_2d::SimpleTerrainAnalyzer, finenav_2d::TerrainAnalyzerBase)

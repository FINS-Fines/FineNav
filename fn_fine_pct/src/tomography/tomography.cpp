#include <iostream>
#include "Tomography.hpp"

namespace finenav_2d {

Tomography::Tomography(const TomographyConfig& config) {
    config_ = config;
}

void Tomography::setInputCloud(const PointCloud::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "[Tomography] Input cloud is empty or invalid." << std::endl;
        return;
    }
    cloud_ = cloud;
}

void Tomography::startAlgorithm() {
    std::cout << "[Tomography] Process Start." << std::endl;

    initMappingEnv();         // 初始化地图环境
    point2map();              // 点云投影到地图
    computeGradients();       // 计 算梯度
    computeTraversability();  // 计算可通行性
    inflateCosts();           // 代价膨胀
    simplifyLayers();         // 图层简化

    std::cout << "[Tomography] Process Finished." << std::endl;
}

/**
 * @brief 初始化地图
 * @details
 */
void Tomography::initMappingEnv() {

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_, min_pt, max_pt);

    // Set ground height
    min_pt.z = config_.ground_h;

    // Calculate map dimensions
    center_ = {(max_pt.x + min_pt.x) / 2.0f, (max_pt.y + min_pt.y) / 2.0f};
    map_dim_x_ = static_cast<int>(std::ceil((max_pt.x - min_pt.x) / config_.resolution) + 4);
    map_dim_y_ = static_cast<int>(std::ceil((max_pt.y - min_pt.y) / config_.resolution) + 4);
    n_slice_init_ =
        static_cast<int>(std::ceil((max_pt.z - min_pt.z) / config_.slice_dh));  // 初始化时候，切片为n_slice_init_
    slice_h0_ = min_pt.z + config_.slice_dh;                                    // 第一个切片h0的高度

    // Initialize buffers
    clearMap();

    // TODO: 为什么膨胀查找表在这里？
    // Initialize inflation table
    // 预先构建代价膨胀查找表
    int half_inf_k_size = static_cast<int>((config_.safe_margin + config_.inflation) / config_.resolution);
    inf_table_.resize(2 * half_inf_k_size + 1, std::vector<float>(2 * half_inf_k_size + 1, 0.0f));

    for (int i = 0; i < inf_table_.size(); ++i) {
        for (int j = 0; j < inf_table_[0].size(); ++j) {
            float dist = std::sqrt(std::pow(config_.resolution * (i - half_inf_k_size), 2) +
                                   std::pow(config_.resolution * (j - half_inf_k_size), 2));
            inf_table_[i][j] =
                std::clamp(1.0f - (dist - config_.inflation) / (config_.safe_margin + config_.resolution), 0.0f, 1.0f);
        }
    }

    std::cout << "[Tomography] Map initialized."
              << " Dim_x: " << map_dim_x_ << ", Dim_y: " << map_dim_y_ << ", Slices: " << n_slice_init_ << std::endl;
}

void Tomography::clearMap() {
    auto max_rows = map_dim_y_;
    auto max_cols = map_dim_x_;

    // 初始化tomography_
    tomography_.ground.layers.clear();
    tomography_.ceiling.layers.clear();
    tomography_.trav_cost.layers.clear();
    for (int s = 0; s < n_slice_init_; ++s) {
        tomography_.ground.layers.emplace_back(
            Layer::Constant(max_rows, max_cols, std::numeric_limits<float>::lowest()));
        tomography_.ceiling.layers.emplace_back(Layer::Constant(max_rows, max_cols, std::numeric_limits<float>::max()));
        tomography_.trav_cost.layers.emplace_back(Layer::Zero(max_rows, max_cols));
    }

    // 初始化中间变量
    grad_mag_sq_.layers.clear();
    grad_mag_max_.layers.clear();
    trav_cost_.layers.clear();
    inflated_cost_.layers.clear();
    for (int s = 0; s < n_slice_init_; ++s) {
        grad_mag_sq_.layers.emplace_back(Layer::Zero(max_rows, max_cols));
        grad_mag_max_.layers.emplace_back(Layer::Zero(max_rows, max_cols));
        trav_cost_.layers.emplace_back(Layer::Zero(max_rows, max_cols));
        inflated_cost_.layers.emplace_back(Layer::Zero(max_rows, max_cols));
    }
}

/**
 * @brief 输入点云加载到tomography地图
 * @param cloud 输入点云
 * @details 将原始点云中的每个点按照高度分配到对应的地图层，并更新每层的地面高度和天花板高度
 * @details tomography地图本质上就是grid_map_，只是在存储时用z-axis-aligned的mulit-layered方式来存储
 * @details layers_g_ 在该层高度范围内，找到"最高的可站立表面"
 * @details layers_c_ 大于该层高度，找到“最矮的顶部”
 */
void Tomography::point2map() {
    for (const auto& point : *cloud_) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // Calculate indices
        int idx_x = static_cast<int>(std::round((point.x - center_[0]) / config_.resolution)) + map_dim_x_ / 2;
        int idx_y = static_cast<int>(std::round((point.y - center_[1]) / config_.resolution)) + map_dim_y_ / 2;

        // Check bounds
        if (idx_x < 0 || idx_x >= map_dim_x_ || idx_y < 0 || idx_y >= map_dim_y_) {
            continue;
        }

        // Update layers
        for (int s_idx = 0; s_idx < n_slice_init_; ++s_idx) {
            float slice = slice_h0_ + s_idx * config_.slice_dh;
            if (point.z <= slice) {
                tomography_.ground(idx_x, idx_y, s_idx) = std::max(tomography_.ground(idx_x, idx_y, s_idx), point.z);
            } else {
                tomography_.ceiling(idx_x, idx_y, s_idx) = std::min(tomography_.ceiling(idx_x, idx_y, s_idx), point.z);
            }
        }
    }
    // 对地图中每一个点的高度进行中值滤波， 中值滤波的核大小为 config_.kernel_size
    int half_k_size = config_.kernal_size / 2;
    for (int s = 0; s < n_slice_init_; ++s) {
        Layer& ground_layer = tomography_.ground.layers[s];
        Layer& ceiling_layer = tomography_.ceiling.layers[s];
        Layer ground_filtered = ground_layer;
        Layer ceiling_filtered = ceiling_layer;
        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                std::vector<float> ground_neighbors;
                std::vector<float> ceiling_neighbors;
                for (int dy = -half_k_size; dy <= half_k_size; ++dy) {
                    for (int dx = -half_k_size; dx <= half_k_size; ++dx) {
                        int ni = i + dx;
                        int nj = j + dy;
                        if (ni >= 0 && ni < map_dim_x_ && nj >= 0 && nj < map_dim_y_) {
                            if (!std::isnan(ground_layer(nj, ni))) {
                                ground_neighbors.push_back(ground_layer(nj, ni));
                            }
                            if (!std::isnan(ceiling_layer(nj, ni))) {
                                ceiling_neighbors.push_back(ceiling_layer(nj, ni));
                            }
                        }
                    }
                }
                if (!ground_neighbors.empty()) {
                    std::nth_element(ground_neighbors.begin(), ground_neighbors.begin() + ground_neighbors.size() / 2,
                                     ground_neighbors.end());
                    ground_filtered(j, i) = ground_neighbors[ground_neighbors.size() / 2];
                } else {
                    ground_filtered(j, i) = std::numeric_limits<float>::lowest();
                }

                if (!ceiling_neighbors.empty()) {
                    std::nth_element(ceiling_neighbors.begin(),
                                     ceiling_neighbors.begin() + ceiling_neighbors.size() / 2, ceiling_neighbors.end());
                    ceiling_filtered(j, i) = ceiling_neighbors[ceiling_neighbors.size() / 2];
                } else {
                    ceiling_filtered(j, i) = std::numeric_limits<float>::max();
                }
            }
        }
        tomography_.ground.layers[s] = ground_filtered;
        tomography_.ceiling.layers[s] = ceiling_filtered;
    }
}

/**
 * @brief 计算每个slice的梯度
 * @note 双向差分后取最大的坡度
 */
void Tomography::computeGradients() {
    for (int s = 0; s < n_slice_init_; ++s) {
        for (int i = 1; i < map_dim_x_ - 1; ++i) {
            for (int j = 1; j < map_dim_y_ - 1; ++j) {
                // Compute x gradient
                float diff_x1 = tomography_.ground(i, j, s) - tomography_.ground(i - 1, j, s);
                float diff_x2 = tomography_.ground(i + 1, j, s) - tomography_.ground(i, j, s);
                float diff_x_sq = std::max(diff_x1 * diff_x1, diff_x2 * diff_x2);

                // Compute y gradient
                float diff_y1 = tomography_.ground(i, j, s) - tomography_.ground(i, j - 1, s);
                float diff_y2 = tomography_.ground(i, j + 1, s) - tomography_.ground(i, j, s);
                float diff_y_sq = std::max(diff_y1 * diff_y1, diff_y2 * diff_y2);

                // Store results
                grad_mag_sq_(i, j, s) = (diff_x_sq + diff_y_sq) / (2 * config_.resolution * config_.resolution);
                // grad_mag_max_(i, j, s) = std::max(diff_x_sq, diff_y_sq)/ (config_.resolution * config_.resolution);

                // if (grad_mag_max_(i, j, s) >= 0.0000 && grad_mag_max_(i, j, s) < 0.30) {
                //     printf("diff_x_sq: %.3f, diff_y_sq: %.3f\n", diff_x_sq, diff_y_sq);
                //     printf("slice %d, cell (%d, %d), grad_mag_max: %.3f\n", s, i, j, grad_mag_max_(i, j, s));
                // }
            }
        }
    }
}  // TODO 之前理解错了

/**
 * @brief 遍历每一个栅格，地形分析
 * @details 维度1：检查机器人是否能通过高度，惩罚过小的interval
 * @details 维度2：检查斜坡坡度是否能上
 * @details 维度3：检查障碍物是否可以跨越，这里需要检查支撑面是否足够
 *
 *
 */
// TODO: 重写COST逻辑
void Tomography::computeTraversability() {

    float tan_max_sq = std::tan(config_.slope_max) * std::tan(config_.slope_max);
    printf("Max traversable slope (tan^2): %.3f\n", tan_max_sq);
    for (int s = 0; s < n_slice_init_; ++s) {
        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                float interval = tomography_.ceiling(i, j, s) - tomography_.ground(i, j, s);

                // Check minimum interval
                // 机器人无法通过的视为障碍物
                if (interval < config_.interval_min) {
                    trav_cost_(i, j, s) = config_.cost_barrier;
                    continue;
                }

                // 此处存在问题
                // Add cost based on slope
                float tan_theta_sq = grad_mag_sq_(i, j, s);

                if (tan_theta_sq <= tan_max_sq) {
                    trav_cost_(i, j, s) += config_.slope_cost_ratio * (tan_theta_sq / tan_max_sq);
                    // Grad_map_sq_ 为  单位距离内的高度变化率的平方
                } else {
                    trav_cost_(i, j, s) = config_.cost_barrier;
                    continue;
                }
            }
        }
    }
}

/**
 * @brief 计算膨胀层代价
 */
void Tomography::inflateCosts() {
    int half_inf_k_size = static_cast<int>((config_.safe_margin + config_.inflation) / config_.resolution);
    for (int s = 0; s < n_slice_init_; ++s) {
        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                float max_cost = 0.0f;
                for (int dy = -half_inf_k_size; dy <= half_inf_k_size; ++dy) {
                    for (int dx = -half_inf_k_size; dx <= half_inf_k_size; ++dx) {
                        int ni = i + dx;
                        int nj = j + dy;
                        if (ni >= 0 && ni < map_dim_x_ && nj >= 0 && nj < map_dim_y_) {
                            max_cost = std::max(max_cost, trav_cost_(ni, nj, s) *
                                                              inf_table_[dy + half_inf_k_size][dx + half_inf_k_size]);
                        }
                    }
                }
                inflated_cost_(i, j, s) = max_cost;
            }
        }
    }
}

/**
 * @brief 根据一定的逻辑，挑选出对于规划器关键的层（提前剪枝）
 */
void Tomography::simplifyLayers() {
    std::vector<int> idx_simp_;  // 存储简化后的层索引
    idx_simp_.push_back(0);

    if (n_slice_init_ > 1) {
        int l_idx = 0, m_idx = 1;
        while (m_idx < n_slice_init_ - 2) {
            bool has_unique = false;
            for (int i = 0; i < map_dim_x_; ++i) {
                for (int j = 0; j < map_dim_y_; ++j) {
                    // 四个独特性条件检查
                    bool mask_l_g = tomography_.ground(i, j, m_idx) - tomography_.ground(i, j, l_idx) > 0;
                    bool mask_l_t = inflated_cost_(i, j, l_idx) > inflated_cost_(i, j, m_idx);
                    bool mask_u_g = (tomography_.ground(i, j, m_idx + 1) - tomography_.ground(i, j, m_idx)) > 0;
                    bool mask_t = inflated_cost_(i, j, m_idx) < config_.cost_barrier;
                    if ((mask_l_g || mask_l_t) && mask_u_g && mask_t) {
                        has_unique = true;
                        break;
                    }
                }
                if (has_unique)
                    break;
            }
            if (has_unique) {
                idx_simp_.push_back(m_idx);
                l_idx = m_idx;
            }
            m_idx++;
        }
        idx_simp_.push_back(m_idx);
    }

    // 直接在原有layers_中更新简化层数据
    for (size_t k = 0; k < idx_simp_.size(); ++k) {
        int s = idx_simp_[k];  // unique layer index

        // 更新 trav_cost
        tomography_.trav_cost.layers[k] = inflated_cost_.layers[s];

        // 更新 ground 和 ceiling，处理无效值
        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                float g_val = tomography_.ground(i, j, s);
                float c_val = tomography_.ceiling(i, j, s);
                tomography_.ground(i, j, k) = (g_val > std::numeric_limits<float>::lowest()) ? g_val : NAN;
                tomography_.ceiling(i, j, k) = (c_val < std::numeric_limits<float>::max()) ? c_val : NAN;
            }
        }
    }

    // 移除多余的层，只保留简化后的层
    tomography_.trav_cost.layers.resize(idx_simp_.size());
    tomography_.ground.layers.resize(idx_simp_.size());
    tomography_.ceiling.layers.resize(idx_simp_.size());

    std::cout << "[Tomography] Simplified layers num: " << idx_simp_.size() << std::endl;
}

}  // namespace finenav_2d

#include "Tomography.hpp"

Tomography::Tomography()
    : Node("pointcloud_tomography"),
      tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)),
      pcd_file_path_("../rsc/pcd/building.pcd"),
      cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    // 文件路径参数
    this->declare_parameter("pcd_file_path", pcd_file_path_);
    pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();

    // 算法参数声明和获取
    this->declare_parameter("resolution", cfg_.resolution);
    this->declare_parameter("slice_dh", cfg_.slice_dh);
    this->declare_parameter("ground_h", cfg_.ground_h);
    this->declare_parameter("half_kernel_size", cfg_.half_kernel_size);
    this->declare_parameter("interval_min", cfg_.interval_min);
    this->declare_parameter("interval_free", cfg_.interval_free);
    this->declare_parameter("slope_max", cfg_.slope_max);
    this->declare_parameter("step_max", cfg_.step_max);
    this->declare_parameter("standable_ratio", cfg_.standable_ratio);
    this->declare_parameter("cost_barrier", cfg_.cost_barrier);
    this->declare_parameter("safe_margin", cfg_.safe_margin);
    this->declare_parameter("inflation", cfg_.inflation);

    // 获取参数值
    cfg_.resolution = this->get_parameter("resolution").as_double();
    cfg_.slice_dh = this->get_parameter("slice_dh").as_double();
    cfg_.ground_h = this->get_parameter("ground_h").as_double();
    cfg_.half_kernel_size = this->get_parameter("half_kernel_size").as_int();
    cfg_.interval_min = this->get_parameter("interval_min").as_double();
    cfg_.interval_free = this->get_parameter("interval_free").as_double();
    cfg_.slope_max = this->get_parameter("slope_max").as_double();
    cfg_.step_max = this->get_parameter("step_max").as_double();
    cfg_.standable_ratio = this->get_parameter("standable_ratio").as_double();
    cfg_.cost_barrier = this->get_parameter("cost_barrier").as_double();
    cfg_.safe_margin = this->get_parameter("safe_margin").as_double();
    cfg_.inflation = this->get_parameter("inflation").as_double();

    // 初始化发布器
    pub_costmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
    pub_gradients_ = this->create_publisher<geometry_msgs::msg::PoseArray>("gradients", 10);
    pub_tomography_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "tomography_results",
        rclcpp::SensorDataQoS().reliable());

    // 发布静态TF
    publishStaticTransform();

    // 加载和处理数据
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    loadPCD();
    processPointCloud();
}

void Tomography::publishStaticTransform() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 1.0;  // 有效的四元数
    tf_broadcaster_->sendTransform(transform);
    RCLCPP_INFO(this->get_logger(), "Static TF published: map → base_link");
}

void Tomography::loadPCD() {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s",
                    pcd_file_path_.c_str());
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded PCD file: %s with %ld points",
               pcd_file_path_.c_str(), cloud_->size());
}

void Tomography::processPointCloud() {
    RCLCPP_INFO(this->get_logger(), "START::Tomography Processing");

    // 1. 执行必要的数据处理流程
    initMappingEnv();          // 初始化地图环境
    point2map(cloud_);         // 点云投影到地图
    computeGradients();        // 计算梯度
    computeTraversability();   // 计算可通行性
    inflateCosts();            // 代价膨胀
    simplifyLayers();          // 图层简化

    costmap_layers_.clear();
    for (size_t k = 0; k < idx_simp_.size(); ++k) {
        CostmapLayer layer;
        layer.costs = layers_t_simp_[k];
        layer.min_height = slice_h0_ + idx_simp_[k] * cfg_.slice_dh;
        layer.max_height = layer.min_height + cfg_.slice_dh;
        costmap_layers_.push_back(layer);
    }

    // 2. 核心发布操作
    auto start_time = std::chrono::steady_clock::now();

    // 确保在发布前数据有效
    if (layers_g_simp_.empty() || map_dim_x_ <= 0 || map_dim_y_ <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid data dimensions for publishing");
        return;
    }

    // 执行发布
    try {
        publishTomographyResults();
        publishCostmapAndGradients();

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        RCLCPP_DEBUG(this->get_logger(),
                    "Publishing completed in %ld ms",
                    duration.count());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                    "Publishing failed: %s",
                    e.what());
    }

    processing_complete_ = true;
    RCLCPP_INFO(this->get_logger(), "END::Tomography Processing======================================");
}

/**
 * @brief 初始化地图
 * @details
 */
void Tomography::initMappingEnv() {
    // Find min and max points
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_, min_pt, max_pt);

    // Set ground height
    min_pt.z = cfg_.ground_h;

    // Calculate map dimensions
    center_ = { (max_pt.x + min_pt.x) / 2.0f, (max_pt.y + min_pt.y) / 2.0f };
    map_dim_x_ = static_cast<int>(std::ceil((max_pt.x - min_pt.x) / cfg_.resolution) + 4);
    map_dim_y_ = static_cast<int>(std::ceil((max_pt.y - min_pt.y) / cfg_.resolution) + 4);
    n_slice_init_ = static_cast<int>(std::ceil((max_pt.z - min_pt.z) / cfg_.slice_dh)); // 初始化时候，切片为n_slice_init_
    slice_h0_ = min_pt.z + cfg_.slice_dh; // 第一个切片h0的高度

    // Initialize buffers
    // 初始化底涂层
    clearMap();

    // Initialize inflation table
    // 预先构建代价膨胀查找表
    int half_inf_k_size = static_cast<int>((cfg_.safe_margin + cfg_.inflation) / cfg_.resolution);
    inf_table_.resize(2 * half_inf_k_size + 1, std::vector<float>(2 * half_inf_k_size + 1, 0.0f));

    for (int i = 0; i < inf_table_.size(); ++i) {
        for (int j = 0; j < inf_table_[0].size(); ++j) {
            float dist = std::sqrt(
                std::pow(cfg_.resolution * (i - half_inf_k_size), 2) +
                std::pow(cfg_.resolution * (j - half_inf_k_size), 2));
            inf_table_[i][j] = std::clamp(
                1.0f - (dist - cfg_.inflation) / (cfg_.safe_margin + cfg_.resolution),
                0.0f, 1.0f);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Map initialized. Dim_x: %d, Dim_y: %d, Slices: %d",
        map_dim_x_, map_dim_y_, n_slice_init_);
}

void Tomography::clearMap() {
    // Initialize layers with default values
    layers_g_.assign(n_slice_init_,
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, -1e6f)));
    layers_c_.assign(n_slice_init_,
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 1e6f)));
    grad_mag_sq_.assign(n_slice_init_,
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 0.0f)));
    grad_mag_max_.assign(n_slice_init_,
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 0.0f)));
    trav_cost_.assign(n_slice_init_,
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 0.0f)));
    inflated_cost_.assign(n_slice_init_,
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 0.0f)));
}

/**
 * @brief 输入点云加载到tomography地图
 * @param cloud 输入点云
 * @details 将原始点云中的每个点按照高度分配到对应的地图层，并更新每层的地面高度和天花板高度
 * @details tomography地图本质上就是grid_map_，只是在存储时用z-axis-aligned的mulit-layered方式来存储
 * @details layers_g_ 在该层高度范围内，找到"最高的可站立表面"
 * @details layers_c_ 大于该层高度，找到“最矮的顶部”
 */
void Tomography::point2map(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    for (const auto& point : *cloud) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // Calculate indices
        int idx_x = static_cast<int>(std::round((point.x - center_[0]) / cfg_.resolution)) + map_dim_x_ / 2;
        int idx_y = static_cast<int>(std::round((point.y - center_[1]) / cfg_.resolution)) + map_dim_y_ / 2;

        // Check bounds
        if (idx_x < 0 || idx_x >= map_dim_x_ || idx_y < 0 || idx_y >= map_dim_y_) {
            continue;
        }

        // Update layers
        for (int s_idx = 0; s_idx < n_slice_init_; ++s_idx) {
            float slice = slice_h0_ + s_idx * cfg_.slice_dh;
            if (point.z <= slice) {
                layers_g_[s_idx][idx_x][idx_y] = std::max(layers_g_[s_idx][idx_x][idx_y], point.z);
            } else {
                layers_c_[s_idx][idx_x][idx_y] = std::min(layers_c_[s_idx][idx_x][idx_y], point.z);
            }
        }
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
                float diff_x1 = layers_g_[s][i][j] - layers_g_[s][i-1][j];
                float diff_x2 = layers_g_[s][i+1][j] - layers_g_[s][i][j];
                float diff_x_sq = std::max(diff_x1 * diff_x1, diff_x2 * diff_x2);

                // Compute y gradient
                float diff_y1 = layers_g_[s][i][j] - layers_g_[s][i][j-1];
                float diff_y2 = layers_g_[s][i][j+1] - layers_g_[s][i][j];
                float diff_y_sq = std::max(diff_y1 * diff_y1, diff_y2 * diff_y2);

                // Store results
                grad_mag_sq_[s][i][j] = diff_x_sq + diff_y_sq;
                grad_mag_max_[s][i][j] = std::max(diff_x_sq, diff_y_sq);
            }
        }
    }
}

/**
 * @brief 遍历每一个栅格，地形分析
 * @details 维度1：检查机器人是否能通过高度，惩罚过小的interval
 * @details 维度2：检查斜坡坡度是否能上
 * @details 维度3：检查障碍物是否可以跨越，这里需要检查支撑面是否足够
 *
 *
 */
void Tomography::computeTraversability() {
    float step_stand = 1.2f * cfg_.resolution * std::tan(cfg_.slope_max * M_PI / 180.0f);
    float step_stand_sq = step_stand * step_stand;
    float step_cross_sq = cfg_.step_max * cfg_.step_max;
    int standable_th = static_cast<int>(cfg_.standable_ratio *
        (2 * cfg_.half_kernel_size + 1) * (2 * cfg_.half_kernel_size + 1)) - 1;

    for (int s = 0; s < n_slice_init_; ++s) {
        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {

                float interval = layers_c_[s][i][j] - layers_g_[s][i][j];

                // Check minimum interval
                if (interval < cfg_.interval_min) {
                    trav_cost_[s][i][j] = cfg_.cost_barrier;
                    continue;
                }

                // Add cost based on interval
                trav_cost_[s][i][j] += std::max(0.0f, 20.0f * (cfg_.interval_free - interval));

                // Check standable condition
                if (grad_mag_sq_[s][i][j] <= step_stand_sq) {
                    trav_cost_[s][i][j] += 15.0f * grad_mag_sq_[s][i][j] / step_stand_sq;
                    continue;
                }

                // Check crossable condition
                if (grad_mag_max_[s][i][j] <= step_cross_sq) {
                    int standable_grids = 0;

                    // Count standable grids in neighborhood
                    for (int dy = -cfg_.half_kernel_size; dy <= cfg_.half_kernel_size; ++dy) {
                        for (int dx = -cfg_.half_kernel_size; dx <= cfg_.half_kernel_size; ++dx) {
                            int ni = i + dx;
                            int nj = j + dy;

                            if (ni < 0 || ni >= map_dim_x_ || nj < 0 || nj >= map_dim_y_) {
                                continue;
                            }

                            if (grad_mag_sq_[s][ni][nj] < step_stand_sq) {
                                standable_grids++;
                            }
                        }
                    }

                    if (standable_grids < standable_th) {
                        trav_cost_[s][i][j] = cfg_.cost_barrier;
                    } else {
                        trav_cost_[s][i][j] += 20.0f * grad_mag_max_[s][i][j] / step_cross_sq;
                    }
                } else {
                    trav_cost_[s][i][j] = cfg_.cost_barrier;
                }
            }
        }
    }
}

/**
 * @brief 计算膨胀代价
 */
void Tomography::inflateCosts() {
    int half_inf_k_size = static_cast<int>((cfg_.safe_margin + cfg_.inflation) / cfg_.resolution);

    for (int s = 0; s < n_slice_init_; ++s) {
        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                int counter = 0;
                float max_cost = 0.0f;

                for (int dy = -half_inf_k_size; dy <= half_inf_k_size; ++dy) {
                    for (int dx = -half_inf_k_size; dx <= half_inf_k_size; ++dx) {
                        int ni = i + dx;
                        int nj = j + dy;

                        if (ni >= 0 && ni < map_dim_x_ && nj >= 0 && nj < map_dim_y_) {
                            max_cost = std::max(max_cost, trav_cost_[s][ni][nj] * inf_table_[dy + half_inf_k_size][dx + half_inf_k_size]);
                        }
                        counter++;
                    }
                }

                inflated_cost_[s][i][j] = max_cost;
            }
        }
    }
}

/**
 * @brief 根据一定的逻辑，挑选出对于规划器关键的层（提前剪枝）
 * @todo 我实在不懂为什么在这里还更改了如此多的属性
 */
void Tomography::simplifyLayers() {
    idx_simp_.clear();
    idx_simp_.push_back(0);

    if (n_slice_init_ > 1) {
        int l_idx = 0, m_idx = 1;

        while (m_idx < n_slice_init_ - 2) {
            bool has_unique = false;

            // 从最底层开始
            for (int i = 0; i < map_dim_x_; ++i) {
                for (int j = 0; j < map_dim_y_; ++j) {
                    // 四个独特性条件检查
                    bool mask_l_g = layers_g_[m_idx][i][j] - layers_g_[l_idx][i][j] > 0; // 当前层相比上一个保留层有更高的地面
                    bool mask_l_t = inflated_cost_[l_idx][i][j] > inflated_cost_[m_idx][i][j]; // 当前层的通行代价比上一个保留层更低
                    bool mask_u_g = (layers_g_[m_idx+1][i][j] - layers_g_[m_idx][i][j]) > 0; // 当前层与上一层之间有明显的高度差异
                    bool mask_t = inflated_cost_[m_idx][i][j] < cfg_.cost_barrier; // 当前层不是完全的障碍区域

                    if ((mask_l_g || mask_l_t) && mask_u_g && mask_t) {
                        has_unique = true;
                        break;
                    }
                }
                if (has_unique) break;
            }

            if (has_unique) {
                idx_simp_.push_back(m_idx);
                l_idx = m_idx;
            }
            m_idx++;
        }

        idx_simp_.push_back(m_idx);
    }

    // Prepare simplified layers
    layers_t_simp_.resize(idx_simp_.size(),
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 0.0f)));
    layers_g_simp_.resize(idx_simp_.size(),
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 0.0f)));
    layers_c_simp_.resize(idx_simp_.size(),
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 0.0f)));
    trav_grad_x_.resize(idx_simp_.size(),
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 0.0f)));
    trav_grad_y_.resize(idx_simp_.size(),
        std::vector<std::vector<float>>(map_dim_x_,
            std::vector<float>(map_dim_y_, 0.0f)));

    // Copy data to simplified layers
    // TODO: 原本的版本似乎没有copy
    for (size_t k = 0; k < idx_simp_.size(); ++k) {
        int s = idx_simp_[k];
        layers_t_simp_[k] = inflated_cost_[s];

        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                layers_g_simp_[k][i][j] = layers_g_[s][i][j] > -1e6f ? layers_g_[s][i][j] : NAN;
                layers_c_simp_[k][i][j] = layers_c_[s][i][j] < 1e6f ? layers_c_[s][i][j] : NAN;
            }
        }
    }

    // Compute gradients for simplified layers
    for (size_t k = 0; k < idx_simp_.size(); ++k) {
        for (int i = 1; i < map_dim_x_ - 1; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                    trav_grad_x_[k][i][j] = layers_t_simp_[k][i+1][j] - layers_t_simp_[k][i-1][j];
            }
        }

        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 1; j < map_dim_y_ - 1; ++j) {
                    trav_grad_y_[k][i][j] = layers_t_simp_[k][i][j+1] - layers_t_simp_[k][i][j-1];
            }
        }
    }


    RCLCPP_INFO(this->get_logger(), "Simplified to %zu layers", idx_simp_.size());
}

void Tomography::publishTomographyResults() {
    // 1. 创建带颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
    colored_cloud.reserve(map_dim_x_ * map_dim_y_ * layers_g_simp_.size());

    // 2. 填充点云数据（按高度着色）
    for (size_t k = 0; k < layers_g_simp_.size(); ++k) {
        // 分层着色
        float hue = static_cast<float>(k) / layers_g_simp_.size() * 360.0f;

        // 将HSV转换为RGB (H:0-360, S:1.0, V:1.0)
        float c = 1.0f;
        float x = c * (1.0f - fabs(fmod(hue / 60.0f, 2.0f) - 1.0f));
        float m = 0.0f;

        float r, g, b;
        if (hue < 60) {
            r = c; g = x; b = 0;
        } else if (hue < 120) {
            r = x; g = c; b = 0;
        } else if (hue < 180) {
            r = 0; g = c; b = x;
        } else if (hue < 240) {
            r = 0; g = x; b = c;
        } else if (hue < 300) {
            r = x; g = 0; b = c;
        } else {
            r = c; g = 0; b = x;
        }

        // 转换为0-255范围
        uint8_t R = static_cast<uint8_t>((r + m) * 255);
        uint8_t G = static_cast<uint8_t>((g + m) * 255);
        uint8_t B = static_cast<uint8_t>((b + m) * 255);

        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                if (std::isnan(layers_g_simp_[k][i][j])) continue;

                pcl::PointXYZRGB point;
                // 坐标转换
                point.x = center_[0] + (i - map_dim_x_/2) * cfg_.resolution;
                point.y = center_[1] + (j - map_dim_y_/2) * cfg_.resolution;
                point.z = layers_g_simp_[k][i][j];

                // set color option
                point.r = R;
                point.g = G;
                point.b = B;

                colored_cloud.push_back(point);
            }
        }
    }

    // 3. 转换并发布点云
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(colored_cloud, output);
    output.header.frame_id = "map";  // 必须与TF一致
    output.header.stamp = this->now();
    pub_tomography_->publish(output);

    RCLCPP_INFO(this->get_logger(), "Published %zu points to /tomography_results",
               colored_cloud.size());

    const std::string pcd_path = "tomography_results.pcd";
    if (pcl::io::savePCDFileBinary(pcd_path, colored_cloud) == 0) {
        RCLCPP_INFO(this->get_logger(), "Successfully saved to %s", pcd_path.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file!");
    }
}

void Tomography::publishCostmapAndGradients() {
    for (size_t layer = 0; layer < layers_g_simp_.size(); ++layer) {
        // 1. 发布代价地图
        nav_msgs::msg::OccupancyGrid costmap_msg;
        costmap_msg.header.frame_id = "map";
        costmap_msg.header.stamp = this->now();
        costmap_msg.info.resolution = cfg_.resolution;
        costmap_msg.info.width = map_dim_x_;
        costmap_msg.info.height = map_dim_y_;
        costmap_msg.info.origin.position.x = center_[0] - (map_dim_x_ / 2) * cfg_.resolution;
        costmap_msg.info.origin.position.y = center_[1] - (map_dim_y_ / 2) * cfg_.resolution;
        costmap_msg.info.origin.orientation.w = 1.0;

        costmap_msg.data.resize(map_dim_x_ * map_dim_y_);

        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                // 检查是否为无效点
                if (std::isnan(layers_g_simp_[layer][i][j])) {
                    costmap_msg.data[j * map_dim_x_ + i] = OCCUPIED;
                }
                // 检查是否为无穷大代价（完全障碍）
                else if (inflated_cost_[layer][i][j] >= FLOAT_INFINITY) {
                    costmap_msg.data[j * map_dim_x_ + i] = OCCUPIED;
                }
                // 自由空间
                else if (inflated_cost_[layer][i][j] <= 0.0f) {
                    costmap_msg.data[j * map_dim_x_ + i] = FREE;
                }
                // 正常代价范围
                else {
                    costmap_msg.data[j * map_dim_x_ + i] = static_cast<int8_t>(
                        std::min(100.0f,
                                inflated_cost_[layer][i][j] * 100 / cfg_.cost_barrier)
                    );
                }
            }
        }
        pub_costmap_->publish(costmap_msg);

        // 2. 发布梯度（仅处理有效点）
        geometry_msgs::msg::PoseArray gradients_msg;
        gradients_msg.header = costmap_msg.header;

        for (int i = 1; i < map_dim_x_ - 1; ++i) {
            for (int j = 1; j < map_dim_y_ - 1; ++j) {
                // 跳过无效点和无穷大代价点
                if (std::isnan(layers_g_simp_[layer][i][j])) {
                    continue;
                }

                // 检查是否为可通行区域
                if (inflated_cost_[layer][i][j] < cfg_.cost_barrier * 0.5f) {
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = center_[0] + (i - map_dim_x_/2) * cfg_.resolution;
                    pose.position.y = center_[1] + (j - map_dim_y_/2) * cfg_.resolution;
                    pose.position.z = layers_g_simp_[layer][i][j];

                    pose.orientation.x = trav_grad_x_[layer][i][j];
                    pose.orientation.y = trav_grad_y_[layer][i][j];
                    pose.orientation.z = 0;
                    pose.orientation.w = 1.0;

                    gradients_msg.poses.push_back(pose);
                }
            }
        }
        pub_gradients_->publish(gradients_msg);

        // 添加延迟避免数据拥堵
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
}

float Tomography::getGroundHeight(int layer, int x, int y) const {
    if (layer >= 0 && layer < static_cast<int>(layers_g_simp_.size()) &&
        x >= 0 && x < map_dim_x_ && y >= 0 && y < map_dim_y_) {
        return layers_g_simp_[layer][x][y];
        }
    return NAN;
}

float Tomography::getInflatedCost(int layer, int x, int y) const {
    if (layer >= 0 && layer < static_cast<int>(inflated_cost_.size()) &&
        x >= 0 && x < map_dim_x_ && y >= 0 && y < map_dim_y_) {
        return inflated_cost_[layer][x][y];
        }
    return std::numeric_limits<float>::max();
}

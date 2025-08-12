#include "fn_fine_pct/Tomography.hpp"
#include "fn_fine_pct/Config.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include "rclcpp/logging.hpp"
#include <pcl/common/common.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <pcl/conversions.h>

Tomography::Tomography()
    : Node("pointcloud_tomography"),
      tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)),
      pcd_file_path_("rsc/pcd/building.pcd"),
      cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    // 参数初始化
    this->declare_parameter("pcd_file_path", pcd_file_path_);
    pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();

    // 初始化发布器
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

    // 2. 核心发布操作
    auto start_time = std::chrono::steady_clock::now();

    // 确保在发布前数据有效
    if (layers_g_simp_.empty() || map_dim_x_ <= 0 || map_dim_y_ <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid data dimensions for publishing");
        return;
    }

    // 执行发布
    try {
        publishTomographyResults();  // 调用你的发布函数

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

    RCLCPP_INFO(this->get_logger(), "END::Tomography Processing");
}

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
    n_slice_init_ = static_cast<int>(std::ceil((max_pt.z - min_pt.z) / cfg_.slice_dh));
    slice_h0_ = min_pt.z + cfg_.slice_dh;

    // Initialize buffers
    clearMap();

    // Initialize inflation table
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

void Tomography::simplifyLayers() {
    idx_simp_.clear();
    idx_simp_.push_back(0);

    if (n_slice_init_ > 1) {
        int l_idx = 0, m_idx = 1;

        while (m_idx < n_slice_init_ - 2) {
            bool has_unique = false;

            for (int i = 0; i < map_dim_x_; ++i) {
                for (int j = 0; j < map_dim_y_; ++j) {
                    bool mask_l_g = layers_g_[m_idx][i][j] - layers_g_[l_idx][i][j] > 0;
                    bool mask_l_t = inflated_cost_[l_idx][i][j] > inflated_cost_[m_idx][i][j];
                    bool mask_u_g = (layers_g_[m_idx+1][i][j] - layers_g_[m_idx][i][j]) > 0;
                    bool mask_t = inflated_cost_[m_idx][i][j] < cfg_.cost_barrier;

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
        for (int i = 0; i < map_dim_x_; ++i) {
            for (int j = 0; j < map_dim_y_; ++j) {
                if (std::isnan(layers_g_simp_[k][i][j])) continue;

                pcl::PointXYZRGB point;
                // 坐标转换（重要！）
                point.x = center_[0] + (i - map_dim_x_/2) * cfg_.resolution;
                point.y = center_[1] + (j - map_dim_y_/2) * cfg_.resolution;
                point.z = layers_g_simp_[k][i][j];

                // 高度映射颜色（可视化关键）
                float height_ratio = (point.z - cfg_.ground_h) / (cfg_.slice_dh * layers_g_simp_.size());
                point.r = static_cast<uint8_t>(255 * height_ratio);
                point.g = static_cast<uint8_t>(255 * (1 - height_ratio));
                point.b = 100;

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
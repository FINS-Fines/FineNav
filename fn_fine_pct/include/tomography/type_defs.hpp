// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_TYPE_DEFS_HPP
#define FINENAV2D_TYPE_DEFS_HPP

using Layer = Eigen::MatrixXf;
using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

/**
 *  @brief tomography算法参数配置
 *  @details 在Tomography初始化时从ROS2参数服务器读取
 */
struct TomographyConfig {
    float resolution = 0.1f;       // Map resolution (meters)
    float slice_dh = 0.5f;         // Height interval between slices // TODO: 它是如何影响的
    float ground_h = 0.0f;         // Ground height
    int half_kernel_size = 7;      // Traversal kernel half-size
    float interval_min = 0.50f;     // Minimum traversable interval
    float interval_free = 0.65f;    // Free space interval
    float slope_max = 0.36f;      // Maximum traversable slope (degrees)
    float step_max = 0.17f;         // Maximum step height
    float standable_ratio = 0.2f;  // Ratio of standable points required
    float cost_barrier = 50.0f;  // Cost for non-traversable areas
    float safe_margin = 0.4f;      // Safe margin around obstacles // TODO:硬安全边界?
    float inflation = 0.2f;        // Inflation radius // TODO:膨胀层？
};


/**
 *  @brief 多层Layer结构
 *  @note x-对应cols，y-对应rows
 */
struct LayerVolume {
    float& operator()(const size_t& x, const size_t& y, const size_t& slice) {
        return layers[slice](y, x);
    }

    const float& operator()(const size_t& x, const size_t& y, const size_t& slice) const {
        return layers[slice](y, x);
    }

    void reset() {
        layers.clear();
    }

    std::vector<Layer> layers;
};

/**
 *  @brief tomography算法输出的数据结构
 */
struct TomographyLayers {
    LayerVolume trav_cost; // Traversability cost layer
    LayerVolume ground; // ground layer
    LayerVolume ceiling; // ceiling layer
};



#endif // FINENAV2D_TYPE_DEFS_HPP

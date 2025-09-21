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


 // Fines 配置使用
 // 0.05 精度下 可以实现一楼较好效果
struct TomographyConfig {
    float resolution = 0.5f;       // Map resolution (meters)
    float slice_dh = 0.8f;         // Height interval between slices // TODO: 它是如何影响的
    float ground_h = 0.0f;         // Ground height
    float interval_min = 0.8f;     // Minimum traversable interval
    float slope_max = 0.34906585f;      // Maximum traversable slope (degrees) 3.1415926/12 = 0.261799388, 3.1415926/6 = 0.523598776 3.1415926/9 = 0.34906585 3.1415926/4 = 0.785398163
    float slope_cost_ratio = 0.0f; // Slope cost ratio 这个参数用来代表机器人爬坡的损耗 
    float cost_barrier = 50.0f;  // Cost for non-traversable areas
    float safe_margin = 0.15f;      // Safe margin around obstacles // TODO:硬安全边界?
    float inflation = 0.05f;        // Inflation radius // TODO:膨胀层？
    int kernal_size = 0;        // 中值滤波的核大小
};
// struct TomographyConfig {
//     float resolution = 0.02f;       // Map resolution (meters)
//     float slice_dh = 0.45f;         // Height interval between slices // TODO: 它是如何影响的
//     float ground_h = 0.0f;         // Ground height
//     float interval_min = 0.45f;     // Minimum traversable interval
//     float slope_max = 0.523598776f;      // Maximum traversable slope (degrees) 3.1415926/12 = 0.261799388, 3.1415926/6 = 0.523598776 3.1415926/9 = 0.34906585
//     float slope_cost_ratio = 1.0f; // Slope cost ratio 这个参数用来代表机器人爬坡的损耗 
//     float cost_barrier = 50.0f;  // Cost for non-traversable areas
//     float safe_margin = 0.05f;      // Safe margin around obstacles // TODO:硬安全边界?
//     float inflation = 0.001f;        // Inflation radius // TODO:膨胀层？
//     int kernal_size = 3;        // 中值滤波的核大小
// };
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

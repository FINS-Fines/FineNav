// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_TOMOGRAPHY_HPP
#define FINENAV2D_TOMOGRAPHY_HPP

#include <string>
#include <vector>
#include <limits>
#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "type_defs.hpp"

namespace finenav_2d {
class Tomography{
public:
    Tomography(const TomographyConfig &config = TomographyConfig());

    void setInputCloud(const PointCloud::Ptr& cloud);
    void startAlgorithm();

    /**
     * @brief 获取输出的图层
     * @note layers_.ground和layers_.ceiling中存在NAN，代表不存在ground或ceiling
     */
    const TomographyLayers& getOutputLayers() const { return tomography_; }

    int getMapDimX() const { return map_dim_x_; }
    int getMapDimY() const { return map_dim_y_; }
    float getResolution() const { return config_.resolution; }
    const std::vector<float>& getCenter() const { return center_; }

private:
    // Algorithm functions
    void initMappingEnv();
    void clearMap();
    void point2map();
    void computeGradients();
    void computeTraversability();
    void inflateCosts();
    void simplifyLayers();

    TomographyConfig config_;  // Configuration parameters
    PointCloud::ConstPtr cloud_;

    // Map metadata
    std::vector<float> center_;
    int map_dim_x_{};
    int map_dim_y_{};
    int n_slice_init_{};
    float slice_h0_{};

    // Map layers
    TomographyLayers tomography_;

    // 内部暂存的东西
    LayerVolume grad_mag_sq_;  // Gradient magnitude squared
    LayerVolume grad_mag_max_;  // Max gradient component
    LayerVolume trav_cost_;     // Traversability cost
    LayerVolume inflated_cost_; // Inflated cost
    std::vector<std::vector<float>> inf_table_; // 离线存储的膨胀表

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace finenav_2d

#endif // FINENAV2D_TOMOGRAPHY_HPP

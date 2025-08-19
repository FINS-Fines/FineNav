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

    int getMapDimX() const { return map_dim_x_; }
    int getMapDimY() const { return map_dim_y_; }
    float getResolution() const { return config_.resolution; }
    const std::vector<float>& getCenter() const { return center_; }

private:
    void loadPCD();
    void processPointCloud();

    // Algorithm functions
    void initMappingEnv();
    void clearMap();
    void point2map(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void computeGradients();
    void computeTraversability();
    void inflateCosts();
    void simplifyLayers();

    void publishCostmapAndGradients();

    TomographyConfig config_;  // Configuration parameters

    // Map metadata
    std::vector<float> center_;
    int map_dim_x_;
    int map_dim_y_;
    int n_slice_init_;
    float slice_h0_;

    // Map layers
    std::vector<TomographyLayer> layers_;

    // 内部暂存的东西
    std::vector<Layer> grad_mag_sq_;  // Gradient magnitude squared
    std::vector<Layer> grad_mag_max_;  // Max gradient component
    std::vector<Layer> trav_cost_;     // Traversability cost
    std::vector<Layer> inflated_cost_; // Inflated cost
    std::vector<std::vector<float>> inf_table_; // 离线存储的膨胀表

    // PCD文件路径
    std::string pcd_file_path_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace finenav_2d

#endif // FINENAV2D_TOMOGRAPHY_HPP

// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "terrain_analyzer_base.hpp"

namespace finenav_2d {

class SimpleTerrainAnalyzer : public TerrainAnalyzerBase {
public:
    SimpleTerrainAnalyzer() {}
    ~SimpleTerrainAnalyzer() override = default;

    void configure(const rclcpp::Node::WeakPtr & parent,
        std::string name,
        const TerrainAnalyzerInterface::Ptr &map_interface) override;

    void analyzeTerrain(const float robot_pose_z) override;

private:
    Eigen::ArrayXXf median_filtering(const Eigen::ArrayXXf& input_array, const int kernel_size);

};

} // nanamespace finenav_2d
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "terrain_analyzer_interface.hpp"

namespace finenav_2d {

class TerrainAnalyzerBase
{
public:
    explicit TerrainAnalyzerBase() : node_(), interface_(nullptr) {}
    virtual ~TerrainAnalyzerBase() = default;

    virtual void configure(
        const rclcpp::Node::WeakPtr & parent,
        std::string name,
        const TerrainAnalyzerInterface::Ptr &map_interface) = 0;

    virtual void analyzeTerrain(const float robot_pose_z) = 0;

protected:
    rclcpp::Node::WeakPtr node_;
    std::string name_;
    TerrainAnalyzerInterface::Ptr interface_;
};

} // namespace finenav_2d
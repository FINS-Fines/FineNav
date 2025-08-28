// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "cloud_publish_helper.hpp"
#include "terrain_analyzer_base.hpp"

namespace finenav_2d {

class SimpleTerrainAnalyzer : public TerrainAnalyzerBase {
public:
    SimpleTerrainAnalyzer() {}
    ~SimpleTerrainAnalyzer() override = default;

    void configure(const rclcpp::Node::WeakPtr & parent,
        std::string name,
        const TerrainAnalyzerInterface::Ptr &map_interface) override;

    void analyzeTerrain() override;

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    finenav_utils::CloudPublishHelper pub_helper_;


};

} // nanamespace finenav_2d
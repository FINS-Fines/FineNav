// Copyright (c) 2025. 
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include "terrain_analysis_simple.hpp"


namespace finenav_2d {

void TerrainAnalysisSimple::analyzeTerrain(){

}



} // namespace finenav_2d

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(finenav_2d::TerrainAnalysisSimple, finenav_2d::TerrainAnalysisBase)

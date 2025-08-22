// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "terrain_analysis_base.hpp"

namespace finenav_2d {

TerrainAnalysisBase::TerrainAnalysisBase() : map_interface_(nullptr) {}

void TerrainAnalysisBase::configure(const MapInterface::Ptr &map_interface){
    map_interface_ = map_interface;
}

float TerrainAnalysisBase::getTerrainAttribute(const std::string& attr_name, const Index& index) const {
    return map_interface_->getAttributeFields().at(attr_name, index);
}

void TerrainAnalysisBase::setTerrainAttribute(const std::string& attr_name, const Index& index, const float& value) const {
    map_interface_->getAttributeFields().at(attr_name, index) = value;
}



} // namespace finenav_2d
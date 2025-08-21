// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "terrain_analysis_base.hpp"

namespace finenav_2d {

bool TerrainAnalysisBase::isOccupied(const Index& index) const {
    return map_interface_->isOccupied(index);
}

float TerrainAnalysisBase::terrainAttribute(const std::string& attr_name, const Index& index) const {
    return map_interface_->getAttributeFields().at(attr_name, index);
}

float& TerrainAnalysisBase::terrainAttribute(const std::string& attr_name, const Index& index) {
    return map_interface_->getAttributeFields().at(attr_name, index);
}

} // namespace finenav_2d
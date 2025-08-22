// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include "map_interface.hpp"

namespace finenav_2d {

class TerrainAnalyzerBase
{
public:
    explicit TerrainAnalyzerBase() : map_interface_(nullptr) {}
    virtual ~TerrainAnalyzerBase() = default;

    void configure(const MapInterface::Ptr &map_interface) {
        map_interface_ = map_interface;
    }

    float getTerrainAttribute(const std::string& attr_name, const Index& index) const {
        return map_interface_->getAttributeFields().at(attr_name, index);
    }

    void setTerrainAttribute(const std::string& attr_name, const Index& index, const float& value) const {
        map_interface_->getAttributeFields().at(attr_name, index) = value;
    }

    virtual void analyzeTerrain() = 0;

protected:
    std::shared_ptr<MapInterface> map_interface_;

};

} // namespace finenav_2d
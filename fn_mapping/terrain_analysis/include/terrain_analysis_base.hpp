// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include "map_interface.hpp"

namespace finenav_2d {

class TerrainAnalysisBase
{
public:
    explicit TerrainAnalysisBase();
    virtual ~TerrainAnalysisBase() = default;

    void configure(const MapInterface::Ptr &map_interface);

    inline float getTerrainAttribute(const std::string& attr_name, const Index& index) const;
    inline void setTerrainAttribute(const std::string& attr_name, const Index& index, const float& value) const;

    virtual void analyzeTerrain() = 0;

protected:
    std::shared_ptr<MapInterface> map_interface_;

};

} // namespace finenav_2d
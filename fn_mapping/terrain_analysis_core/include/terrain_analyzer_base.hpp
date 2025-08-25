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

    void configure(const MapInterfaceBase::Ptr &map_interface) {
        map_interface_ = map_interface;
    }

    MapInterfaceBase::Ptr getMapInterface() const { return map_interface_; }

    virtual void analyzeTerrain() = 0;

protected:
    MapInterfaceBase::Ptr map_interface_;

};

} // namespace finenav_2d
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>

#include "terrain_analyzer_interface.hpp"

namespace finenav_2d {

class TerrainAnalyzerBase
{
public:
    explicit TerrainAnalyzerBase() : interface_(nullptr) {}
    virtual ~TerrainAnalyzerBase() = default;

    void configure(const TerrainAnalyzerInterface::Ptr &map_interface) {
        interface_ = map_interface;
    }

    virtual void analyzeTerrain() = 0;

protected:
    TerrainAnalyzerInterface::Ptr interface_;

};

} // namespace finenav_2d
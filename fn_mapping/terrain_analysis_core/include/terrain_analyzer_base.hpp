// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <span>

#include "terrain_data_view.hpp"

namespace finenav_2d {

class TerrainAnalyzerBase
{
public:
    explicit TerrainAnalyzerBase() : terrain_data_view_(nullptr) {}
    virtual ~TerrainAnalyzerBase() = default;

    void configure(const TerrainDataView::Ptr &map_interface) {
        terrain_data_view_ = map_interface;
    }

    virtual void analyzeTerrain() = 0;

protected:
    TerrainDataView::Ptr terrain_data_view_;

};

} // namespace finenav_2d
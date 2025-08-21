// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

namespace finenav_2d {

template <typename MapT>
class TerrainMapAccessor {

public:

  TerrainMapAccessor() = default;

  virtual ~TerrainMapAccessor() = default;

private:
    MapT map_; // 地图数据





};

}
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <octomap/octomap.h>

namespace finenav_2d {


class octomap_server {
public:
    using OcTreeT = octomap::OcTree;

private:
    std::unique_ptr<OcTreeT> octree_;

};


}

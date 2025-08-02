// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <iostream>
#include "grid_map.hpp"

int main(int argc, char** argv) {
    using namespace finenav_2d;
    std::cout << "Hello, World!" << std::endl;
    GridMap<int> grid_map({10.0, 10.0, 10.0}, 0.1, {0.0, 0.0, 0.0});
  return 0;
}
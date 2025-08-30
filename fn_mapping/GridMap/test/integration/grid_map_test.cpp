// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include <chrono>

#include "grid_map.hpp"
#include "compare_vectors.hpp"

using namespace finenav_2d;

TEST(GridMapTest, All) {
    GridMap<uint8_t> grid_map({10.0, 10.0, 2.0}, 0.1);

    EXPECT_EQ(grid_map.getSize(), Size(101, 101, 21)); // 10m / 0.1m = 100 + 1 (for center) = 201

    /******** 模拟初始化来了一帧点云，给原点为中心的2*2*2m范围赋值为true *******/
    for (int x = -20; x <= 20; ++x) {
        for (int y = -20; y <= 20; ++y) {
            for (int z = -20; z <= 20; ++z) {
                grid_map.atPosition({x * 0.05, y * 0.05, z * 0.05}) = 255;
            }
        }
    }

    std::vector<Index> indices_1;
    grid_map.selectCellsByCondition(indices_1, [](const uint8_t& value) {
        return value == 255;
    });
    EXPECT_EQ(indices_1.size(), 21 * 21 * 21); // 2m范围内的栅格数量

    /******* 模拟机器人发生了移动，从原点移动到(0.1,0.1) *******/
    bool isMoved = grid_map.moveTo({0.1, 0.1, 0.1});

    std::vector<Index> indices_2;
    grid_map.selectCellsByCondition(indices_2, [](const uint8_t& value) {
        return value == 255;
    });
    EXPECT_EQ(isMoved, true);
    EXPECT_TRUE(compareVectors(indices_1, indices_2));



}

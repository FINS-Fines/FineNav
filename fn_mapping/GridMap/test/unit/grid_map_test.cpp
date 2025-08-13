// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include "grid_map.hpp"

using namespace finenav_2d;

/**
 * @brief 输入两个std::vector，比较它们的内容是否相同
 */
static bool compareVectors(const std::vector<Index>& vec1, const std::vector<Index>& vec2) {
    auto toTuple = [](const Index& idx) {
        return std::make_tuple(idx.x(), idx.y(), idx.z());
    };

    std::set<std::tuple<int, int, int>> set1, set2;
    for (const auto& idx : vec1) set1.insert(toTuple(idx));
    for (const auto& idx : vec2) set2.insert(toTuple(idx));

    return set1 == set2;
}

TEST(GridMapTest, RayCast) {
    {
        GridMap<bool> grid_map({20.0,20.0,20.0}, 0.5);
        Position end = {-4,-3,-5};
        std::vector<Index> indices;
        std::vector<Index> result =  {
            {4, 0, 0},
            {3, 0, 0},
            {3, 0, -1},
            {3, -1, -1},
            {2, -1, -1},
            {2, -1, -2},
            {1, -1, -2},
            {1, -1, -3},
            {1, -2, -3},
            {0, -2, -3},
            {0, -2, -4},
            {-1, -2, -4},
            {-2, -2, -4},
            {-2, -2, -5},
            {-2, -3, -5},
            {-3, -3, -5},
            {-3, -3, -6},
            {-3, -4, -6},
            {-4, -4, -6},
            {-4, -4, -7},
            {-5, -4, -7},
            {-5, -4, -8},
            {-5, -5, -8},
            {-6, -5, -8},
            {-6, -5, -9},
            {-7, -5, -9},
            {-7, -5, -10},
            {-7, -6, -10},
            {-8, -6, -10}
        };

        grid_map.setOrigin({2,0,0});

        grid_map.rayCast(end,indices);

        EXPECT_TRUE(compareVectors(result,indices));
    }

    {
        GridMap<bool> grid_map({20.0,20.0,20.0}, 1);
        Position end = {0,0,-5};
        std::vector<Index> indices;
        std::vector<Index> result =
        {{0,0,0}, {0,0,-1}, {0,0,-2}, {0,0,-3}, {0,0,-4}, {0,0,-5}}
        ;

        grid_map.setOrigin({0,0,0});

        grid_map.rayCast(end,indices);

        EXPECT_TRUE(compareVectors(result,indices));
    }

    {
        GridMap<bool> grid_map({20.0,20.0,20.0}, 0.5);
        Position end = {4,-3,3};
        std::vector<Index> indices;
        std::vector<Index> result =  {
            {0, 0, 0},
            {1, 0, 0},
            {1, 0, 1},
            {1, -1, 1},
            {2, -1, 1},
            {2, -1, 2},
            {2, -2, 2},
            {3, -2, 2},
            {3, -2, 3},
            {3, -3, 3},
            {4, -3, 3},
            {5, -3, 3},
            {5, -3, 4},
            {5, -4, 4},
            {6, -4, 4},
            {6, -4, 5},
            {6, -5, 5},
            {7, -5, 5},
            {7, -5, 6},
            {7, -6, 6},
            {8, -6, 6}
        };

        grid_map.setOrigin({0,0,0});

        grid_map.rayCast(end,indices);

        EXPECT_TRUE(compareVectors(result,indices));
    }


    {
        GridMap<bool> grid_map({20.0,20.0,20.0}, 0.5);
        Position end = {7,1,5};
        std::vector<Index> indices;
        std::vector<Index> result =  {
            {4, -2, 6},
            {5, -2, 6},
            {6, -2, 6},
            {6, -2, 7},
            {6, -1, 7},
            {7, -1, 7},
            {8, -1, 7},
            {8, -1, 8},
            {8, 0, 8},
            {9, 0, 8},
            {10, 0, 8},
            {11, 0, 8},
            {11, 0, 9},
            {11, 1, 9},
            {12, 1, 9},
            {13, 1, 9},
            {14, 1, 9},
            {14, 1, 10},
            {14, 2, 10}
        };

        grid_map.setOrigin({2,-1,3});

        grid_map.rayCast(end,indices);

        EXPECT_TRUE(compareVectors(result,indices));
    }
}



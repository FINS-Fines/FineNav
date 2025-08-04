// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include "grid_map_math.hpp"

using namespace finenav_2d;


TEST(GridMapMath, WrapIndexToRange) {
    const Size size {11,11,11};
    {
        Index unwrapped_index(0, 0, 0);
        Index result{0, 0, 0};
        auto wrapped_index= wrapIndexToRange(unwrapped_index, size);
        EXPECT_EQ(wrapped_index, result);
    }
    {
        Index unwrapped_index(-1, 4, 5);
        Index result{10, 4, 5};
        auto wrapped_index = wrapIndexToRange(unwrapped_index, size);
        EXPECT_EQ(wrapped_index, result);
    }
    {
        Index unwrapped_index(11, 35, -25);
        Index result{0, 2, 8};
        auto wrapped_index = wrapIndexToRange(unwrapped_index, size);
        EXPECT_EQ(wrapped_index, result);
    }


}

// TEST(GridMapMath, IndexShiftFromPositionShift) {
//     Position position_shift(0.5, -1.5, 2.0);
//     double resolution = 0.1;
//     Index index_shift = getIndexShiftFromPositionShift(position_shift, resolution);
//     EXPECT_EQ(index_shift.x(), 5);
//     EXPECT_EQ(index_shift.y(), -15);
//     EXPECT_EQ(index_shift.z(), 20);
// }
//
// TEST(GridMapMath, PositionShiftFromIndexShift) {
//     Index index_shift(5, -15, 20);
//     double resolution = 0.1;
//     Position position_shift = getPositionShiftFromIndexShift(index_shift, resolution);
//     EXPECT_DOUBLE_EQ(position_shift.x(), 0.5);
//     EXPECT_DOUBLE_EQ(position_shift.y(), -1.5);
//     EXPECT_DOUBLE_EQ(position_shift.z(), 2.0);
// }
//
// TEST(GridMapMath, DataAcessFromIndex) {
//
// }
//
//
// TEST(GridMapMath, DataAcessFromPosition) {
//
// }
//
//
//
// TEST(GridMapMath, CheckIfIndexValid) {
//
// }
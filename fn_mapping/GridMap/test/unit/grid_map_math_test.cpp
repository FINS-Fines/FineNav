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
        EXPECT_EQ(result, wrapIndexToRange(unwrapped_index, size));
    }
    {
        Index unwrapped_index(-1, 4, 5);
        Index result{10, 4, 5};
        EXPECT_EQ(result, wrapIndexToRange(unwrapped_index, size));
    }
    {
        Index unwrapped_index(11, 35, -25);
        Index result{0, 2, 8};
        auto wrapped_index = wrapIndexToRange(unwrapped_index, size);
        EXPECT_EQ(result, wrapIndexToRange(unwrapped_index, size));
    }
}

TEST(GridMapMath, IndexShiftFromPositionShift) {
    const double resolution = 0.1;
    {
        Position position_shift{0.0, 0.0, 0.0};
        Index result{0, 0, 0};
        EXPECT_EQ(result, getIndexShiftFromPositionShift(position_shift, resolution));
    }
    {
        Position position_shift{0.1, 0.4, 0.5};
        Index result{1, 4, 5};
        EXPECT_EQ(result, getIndexShiftFromPositionShift(position_shift, resolution));
    }
    {
        Position position_shift{-1.0, -2.5, -3.0};
        Index result{-10, -25, -30};
        EXPECT_EQ(result, getIndexShiftFromPositionShift(position_shift, resolution));
    }
    {
        Position position_shift{0.151, 0.285, 0.34567};
        Index result{2, 3, 3};
        EXPECT_EQ(result, getIndexShiftFromPositionShift(position_shift, resolution));
    }
    {
        Position position_shift{-0.151, 0.285, -0.34567};
        Index result{-2, 3, -3};
        EXPECT_EQ(result, getIndexShiftFromPositionShift(position_shift, resolution));
    }
    {
        Position position_shift{10.0, 50.0, 100.0};
        Index result{100, 500, 1000};
        EXPECT_EQ(result, getIndexShiftFromPositionShift(position_shift, resolution));
    }
}

TEST(GridMapMath, PositionShiftFromIndexShift) {
    const double resolution = 0.1;
    {
        Index index_shift{1, 4, 5};
        Position result{0.1, 0.4, 0.5};
        EXPECT_TRUE(result.isApprox(getPositionShiftFromIndexShift(index_shift, resolution)));
    }
    {
        Index index_shift{0, 0, 0};
        Position result{0.0, 0.0, 0.0};
        EXPECT_TRUE(result.isApprox(getPositionShiftFromIndexShift(index_shift, resolution)));
    }
    {
        Index index_shift{-10, -25, -30};
        Position result{-1.0, -2.5, -3.0};
        EXPECT_TRUE(result.isApprox(getPositionShiftFromIndexShift(index_shift, resolution)));
    }
    {
        Index index_shift{2, 3, 3};
        Position result{0.2, 0.3, 0.3};
        EXPECT_TRUE(result.isApprox(getPositionShiftFromIndexShift(index_shift, resolution)));
    }
    {
        Index index_shift{-2, 3, -3};
        Position result{-0.2, 0.3, -0.3};
        EXPECT_TRUE(result.isApprox(getPositionShiftFromIndexShift(index_shift, resolution)));
    }
    {
        Index index_shift{100, 500, 1000};
        Position result{10.0, 50.0, 100.0};
        EXPECT_TRUE(result.isApprox(getPositionShiftFromIndexShift(index_shift, resolution)));
    }


}

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
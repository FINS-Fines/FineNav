#include <gtest/gtest.h>
#include <set>
#include <chrono>

#include "grid_map_math.hpp"

using namespace finenav_2d;

/**
 * @brief 通过完全遍历的方式计算两个栅格地图A和B的差集A-B
 */
void baseline(const Index& index_shift, const Size& size, std::vector<Index>& difference_indices){
    // easy version_CORRECT
    // 清空输出向量
    difference_indices.clear();

    // 计算地图的边界
    Index half_size = size / 2;

    Index a_min = -half_size;
    Index a_max = half_size;

    Index b_min = -half_size + index_shift;
    Index b_max = half_size + index_shift;

    if ((b_min.array() > half_size.array()).any() || (b_max.array() < -half_size.array()).any()) {
        for (int x = -half_size.x(); x <= half_size.x(); ++x) {
            for (int y = -half_size.y(); y <= half_size.y(); ++y) {
                for (int z = -half_size.z(); z <= half_size.z(); ++z) {
                    difference_indices.emplace_back(x, y, z);
                }
            }
        }
        return;
    }
    // 遍历地图 A 的所有可能索引
    for (int x = -half_size.x(); x <= half_size.x(); ++x) {
        for (int y = -half_size.y(); y <= half_size.y(); ++y) {
            for (int z = -half_size.z(); z <= half_size.z(); ++z) {
                Index idx_a(x, y, z);

                // 计算对应的地图 B 中的索引（考虑偏移量）
                Index idx_b = idx_a - index_shift;

                // 检查 idx_b 是否在地图 B 内
                bool in_b = ((std::abs(idx_b.x()) <= half_size.x()) &&
                             (std::abs(idx_b.y()) <= half_size.y()) &&
                             (std::abs(idx_b.z()) <= half_size.z()) );

                // 如果 idx_a 在 A 中但对应的 idx_b 不在 B 中，加入差集
                if (!in_b) {
                    difference_indices.push_back(idx_a);
                }
            }
        }
    }
}

/**
 * @brief 输入两个std::vector，比较它们的内容是否相同
 */
bool compareVectors(const std::vector<Index>& vec1, const std::vector<Index>& vec2) {
    auto toTuple = [](const Index& idx) {
        return std::make_tuple(idx.x(), idx.y(), idx.z());
    };

    std::set<std::tuple<int, int, int>> set1, set2;
    for (const auto& idx : vec1) set1.insert(toTuple(idx));
    for (const auto& idx : vec2) set2.insert(toTuple(idx));

    return set1 == set2;
}

TEST(GetDifferenceSetTest, Simple) {
    Size size(11, 11, 11);

    {
    Index index_shift(1, 0, 0);
    std::vector<Index> method_result, baseline_result;

    baseline(index_shift, size, baseline_result);
    getDifferenceSet(index_shift, size, method_result);
    EXPECT_TRUE(compareVectors(method_result, baseline_result));

    // GTEST_LOG_(INFO) << "index nums: " << baseline_result.size();
    // for (const auto& idx : baseline_result) {
    //     GTEST_LOG_(INFO) << idx.transpose();
    // }
    }
    {
        Index index_shift(1, 1, 1);
        std::vector<Index> method_result, baseline_result;

        baseline(index_shift, size, baseline_result);
        getDifferenceSet(index_shift, size, method_result);
        EXPECT_TRUE(compareVectors(method_result, baseline_result));

        // GTEST_LOG_(INFO) << "index nums: " << baseline_result.size();
        // for (const auto& idx : baseline_result) {
        //     GTEST_LOG_(INFO) << idx.transpose();
        // }
    }
    {
        Index index_shift(-1, -1, -1);
        std::vector<Index> method_result, baseline_result;

        baseline(index_shift, size, baseline_result);
        getDifferenceSet(index_shift, size, method_result);
        EXPECT_TRUE(compareVectors(method_result, baseline_result));

        // GTEST_LOG_(INFO) << "index nums: " << baseline_result.size();
        // for (const auto& idx : baseline_result) {
        //     GTEST_LOG_(INFO) << idx.transpose();
        // }
    }
}

TEST(GetDifferenceSetTest, EdgeCase) {
    Size size(11, 11, 11);
    {
        Index index_shift(0, 0, 0);
        std::vector<Index> method_result, baseline_result;

        baseline(index_shift, size, baseline_result);
        getDifferenceSet(index_shift, size, method_result);
        EXPECT_TRUE(compareVectors(method_result, baseline_result));

        // GTEST_LOG_(INFO) << "index nums: " << baseline_result.size();
        // for (const auto& idx : baseline_result) {
        //     GTEST_LOG_(INFO) << idx.transpose();
        // }
    }
    {
        Index index_shift(20, 20, 20);
        std::vector<Index> method_result, baseline_result;

        baseline(index_shift, size, baseline_result);
        getDifferenceSet(index_shift, size, method_result);
        EXPECT_TRUE(compareVectors(method_result, baseline_result));

        // GTEST_LOG_(INFO) << "index nums: " << baseline_result.size();
        // for (const auto& idx : baseline_result) {
        //     GTEST_LOG_(INFO) << idx.transpose();
        // }
    }
}

TEST(GetDifferenceSetTest, LargeMapCase) {
    Size size(101, 101, 51);
    {
        Index index_shift(1, 1, 0);
        std::vector<Index> method_result, baseline_result;

        auto start = std::chrono::high_resolution_clock::now();
        getDifferenceSet(index_shift, size, method_result);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        GTEST_LOG_(INFO) << "getDifferenceSetTest: Done in: " << duration.count() << " us";

        baseline(index_shift, size, baseline_result);
        EXPECT_TRUE(compareVectors(method_result, baseline_result));

        // GTEST_LOG_(INFO) << "index nums: " << baseline_result.size();
        // for (const auto& idx : baseline_result) {
        //     GTEST_LOG_(INFO) << idx.transpose();
        // }
    }
    {
        Index index_shift(-9, -8, -7);
        std::vector<Index> method_result, baseline_result;

        auto start = std::chrono::high_resolution_clock::now();
        getDifferenceSet(index_shift, size, method_result);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        GTEST_LOG_(INFO) << "getDifferenceSetTest: Done in: " << duration.count() << " ms";

        baseline(index_shift, size, baseline_result);
        EXPECT_TRUE(compareVectors(method_result, baseline_result));

        // GTEST_LOG_(INFO) << "index nums: " << baseline_result.size();
        // for (const auto& idx : baseline_result) {
        //     GTEST_LOG_(INFO) << idx.transpose();
        // }
    }
}
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_COMPARE_VECTORS_HPP
#define FINENAV2D_COMPARE_VECTORS_HPP

#include <Eigen/Core>

/**
 * @brief 输入两个std::vector，比较它们的内容是否相同
 */
static bool compareVectors(const std::vector<Eigen::Vector3i>& vec1, const std::vector<Eigen::Vector3i>& vec2) {
    auto toTuple = [](const Eigen::Vector3i& idx) {
        return std::make_tuple(idx.x(), idx.y(), idx.z());
    };

    std::set<std::tuple<int, int, int>> set1, set2;
    for (const auto& idx : vec1) set1.insert(toTuple(idx));
    for (const auto& idx : vec2) set2.insert(toTuple(idx));

    return set1 == set2;
}

#endif //FINENAV2D_COMPARE_VECTORS_HPP

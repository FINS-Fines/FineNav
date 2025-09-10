// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <limits>
#include <algorithm>
#include "grid_map.hpp"
#include <cmath>


namespace finenav_2d {
template <typename T>
GridMap<T>::GridMap(const Length& length, const double& resolution, const Position& origin)
    : length_(length), resolution_(resolution), origin_(origin) {

    inv_resolution_ = 1.0 / resolution;

    size_ = (length.array() * inv_resolution_).ceil().cast<int>();
    size_ = size_.unaryExpr([](int v) {
        return (v % 2 == 0) ? v + 1 : v; // 确保尺寸为奇数
    });

    half_size_ = size_ / 2;

    data_.resize(size_.x() * size_.y() * size_.z(), NAN);
    start_index_ = Index::Zero();
}

template <typename T>
T& GridMap<T>::atPosition(const Position& position) {
    return at(getIndex(position));
}

template <typename T>
T GridMap<T>::atPosition(const Position& position) const {
    return at(getIndex(position));
}

template <typename T>
T& GridMap<T>::at(const Index& index) {
    if (checkIfIndexValid(index, size_, half_size_)) {
        return data_[getBufferIndex(index, size_, half_size_, start_index_)];
    }
    throw std::out_of_range("Accessing grid out of bounds");
}

template <typename T>
T GridMap<T>::at(const Index& index) const {
    if (checkIfIndexValid(index, size_, half_size_)) {
        return data_[getBufferIndex(index, size_, half_size_, start_index_)];
    }
    throw std::out_of_range("Accessing grid out of bounds");
}

template <typename T>
std::span<T> GridMap<T>::getVoxelsAlongZ(const int& x, const int& y) {
    if (std::abs(x) > half_size_.x() || std::abs(y) > half_size_.y()) {
        throw std::out_of_range("Accessing grid out of bounds");
    }

    Index index_start(x, y, -half_size_.z());
    int buffer_start = getBufferIndex(index_start, size_, half_size_, start_index_);
    return std::span<T>(&data_[buffer_start], size_.z());
}

template <typename T>
std::span<const T> GridMap<T>::getVoxelsAlongZ(const int& x, const int& y) const {
    if (std::abs(x) > half_size_.x() || std::abs(y) > half_size_.y()) {
        throw std::out_of_range("Accessing grid out of bounds");
    }

    Index index_start(x, y, -half_size_.z());
    int buffer_start = getBufferIndex(index_start, size_, half_size_, start_index_);
    return std::span<T>(&data_[buffer_start], size_.z());
}

template <typename T>
void GridMap<T>::clear() {
    data_.clear();
    data_.resize(size_.x() * size_.y() * size_.z(), T{});
}

template <typename T>
bool GridMap<T>::moveTo(const Position& position) {
    std::vector<std::pair<Position, T>> removed_region;
    return moveTo(position, false, removed_region);
}

template <typename T>
bool GridMap<T>::moveTo(const Position& position, std::vector<std::pair<Position, T>>& removed_region) {
    return moveTo(position, false, removed_region);
}

template <typename T>
bool GridMap<T>::moveTo(const Position& position, const bool keep_removed, std::vector<std::pair<Position, T>>& removed_region) {
    removed_region.clear();

    // 计算移动了的栅格数
    const auto index_shift = getIndexShiftFromPositionShift(position - origin_, resolution_, inv_resolution_);
    if (index_shift.isZero()) {
        return false; // 没有移动
    }

    // 以移动后的地图作为固定坐标系，获取受移动影响的栅格
    std::vector<Index> indices;
    getDifferenceSet(index_shift, size_, half_size_, indices);

    for (const auto& index : indices) {
        if (!keep_removed) {
            if (checkIfIndexValid(index, size_, half_size_)) {
                data_[getBufferIndex(index, size_, half_size_, start_index_)] = NAN; // 清空数据
            }
        }
        removed_region.emplace_back(getPosition(index), at(index));
    }

    // 更新移动后的地图状态
    const auto aligned_position_shift = getPositionShiftFromIndexShift(index_shift, resolution_, inv_resolution_);
    origin_ += aligned_position_shift;
    start_index_ = wrapIndexToRange(start_index_ + index_shift, size_); // 地图移动时，逻辑原点在缓冲区中的位置同向移动

    return true;
}

template <typename T>
bool GridMap<T>::rayCast(const Position& origin,const Position& end, std::vector<Index>& indices) const {

    indices.clear();

    double min_length = 0.5;
    double max_length = 5.0;
    auto origin_voxel = getIndex(origin);
    auto last_voxel = getIndex(end);
    auto current_voxel = origin_voxel;
//    // 光线超出地图范围，结束
//    if (!checkIfIndexValid(last_voxel, size_, half_size_)) {
//        return false;
//    }

    // 光线终点为原点，结束
    //   if (origin_index == last_index) {
    //       return true;
    //   }

    // TODO:要不要先把原点放进来

    // 初始化
    Vector direction (end - origin);
    Vector inv_direction = direction.cwiseInverse();


    auto length = direction.norm();
    if (length < min_length) {
        return false;
    }
//    if (length > max_length) {
//        return false;
//    }
    //   direction = direction/length;   对方向向量归一化会引入无理数可能导致浮点数精度造成误差

    Eigen::Vector3i tMult = {0,0,0};
    Vector tMax, tDelta;
    Vector tMax_start, tDelta_start;
    Eigen::Vector3i step;

    for (int i = 0; i < 3; ++i) {
        step[i] = sign(direction[i]);

        if (step[i] != 0) {
            double voxelBorder;
            if (step[i] > 0) {
                voxelBorder = (current_voxel[i] + 1) * resolution_;
            } else {
                voxelBorder = current_voxel[i] * resolution_;
            }

            // 计算到下一个边界的参数距离
            tMax_start[i] = (voxelBorder - origin[i]) * inv_direction[i];
			tMax[i] = tMax_start[i];
            // 计算穿越一个完整体素的参数距离
            tDelta_start[i] = resolution_ * std::abs(inv_direction[i]);
            tDelta[i] = tDelta_start[i];
        }
        else{
            tMax_start[i] = std::numeric_limits<double>::infinity();
            tMax[i] = std::numeric_limits<double>::infinity();
            tDelta_start[i] = std::numeric_limits<double>::infinity();
            tDelta[i] = std::numeric_limits<double>::infinity();
        }
    }

    // TODO:需不需要对step为负值的情况做处理
    indices.push_back(current_voxel);
    while(current_voxel != last_voxel) {
        if (!checkIfIndexValid(current_voxel, size_, half_size_)) {
            return false;
        }

        if (tMax.x() < tMax.y()) {
            if (tMax.x() < tMax.z()) {
                current_voxel.x() += step.x();
                tMax.x() += tDelta.x();
            } else {
                current_voxel.z() += step.z();
                tMax.z() += tDelta.z();
            }
        } else {
            if (tMax.y() < tMax.z()) {
                current_voxel.y() += step.y();
                tMax.y() += tDelta.y();
            } else {
                current_voxel.z() += step.z();
                tMax.z() += tDelta.z();
            }
        }

        indices.push_back(current_voxel);
    }


    // 光线投射算法实现
    // 这里可以使用Bresenham算法或其他光线投射算法
    // 需要根据end位置计算出经过的栅格索引，并存储在indices中
    // 返回true表示成功，false表示失败（例如光线超出地图范围）
    return true; // 需要实现具体逻辑
}


template <typename T>
void GridMap<T>::setOrigin(const Position& origin) {
    origin_ = origin;
}

template <typename T>
void GridMap<T>::selectCellsByCondition(std::vector<Index>& indices, std::function<bool(const T&)> condition) const {
    indices.clear();
    for (int x = -half_size_.x(); x <= half_size_.x(); ++x) {
        for (int y = -half_size_.y(); y <= half_size_.y(); ++y) {
            for (int z = -half_size_.z(); z <= half_size_.z(); ++z) {
                Index idx(x, y, z);
                if (condition(at(idx))) {
                    indices.push_back(idx);
                }
            }
        }
    }
}

template <typename T>
Index GridMap<T>::getIndex(const Position& position) const {
    return getIndexFromPosition(position, resolution_, inv_resolution_, origin_);
}

template <typename T>
Position GridMap<T>::getPosition(const Index& index) const {
    return getPositionFromIndex(index, resolution_, inv_resolution_, origin_);
}

template <typename T>
Length GridMap<T>::getLength() const {
    return length_;
}

template <typename T>
Size GridMap<T>::getSize() const {
    return size_;
}

template <typename T>
double GridMap<T>::getResolution() const {
    return resolution_;
}

template <typename T>
Position GridMap<T>::getOrigin() const {
    return origin_;
}

template <typename T>
Index GridMap<T>::getMinIndex() const{
    return -half_size_;
}

template <typename T>
Index GridMap<T>::getMaxIndex() const{
    return half_size_;
}

template <typename T>
bool GridMap<T>::isInside(const Position& position) const {
    return checkIfIndexValid(getIndex(position), size_, half_size_);
}

template <typename T>
bool GridMap<T>::isInside(const Index& index) const {
    return checkIfIndexValid(index, size_, half_size_);
}

} // namespace finenav_2d


// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef GRID_MAP_IMPL_HPP
#define GRID_MAP_IMPL_HPP

#include "grid_map.hpp"

namespace finenav_2d {

template <typename T>
GridMap<T>::GridMap(const Length& length, const double& resolution, const Position& origin)
    : length_(length), resolution_(resolution), origin_(origin) {

    size_ = (length.array() / resolution).ceil().cast<int>();
    size_ = size_.unaryExpr([](int v) {
        return (v % 2 == 0) ? v + 1 : v; // 确保尺寸为奇数
    });

    data_.resize(size_.x() * size_.y() * size_.z(), T{});
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
    if (checkIfIndexValid(index, size_)) {
        return data_[getBufferIndex(index, size_, start_index_)];
    }
    throw std::out_of_range("Accessing grid out of bounds");
}

template <typename T>
T GridMap<T>::at(const Index& index) const {
    if (checkIfIndexValid(index, size_)) {
        return data_[getBufferIndex(index, size_, start_index_)];
    }
    throw std::out_of_range("Accessing grid out of bounds");
}

template <typename T>
void GridMap<T>::clear() {
    data_.clear();
    data_.resize(size_.x() * size_.y() * size_.z(), T{});
}

template <typename T>
bool GridMap<T>::moveTo(const Position& position) {
    std::vector<Index> indices;
    return moveTo(position, false, indices);
}

template <typename T>
bool GridMap<T>::moveTo(const Position& position, bool keep_removed, std::vector<Index>& indices) {

    // // 计算移动了的栅格数
    // const auto index_shift = getIndexShiftFromPositionShift(position - origin_, resolution_);
    // if (index_shift.isZero()) {
    //     return false; // 没有移动
    // }
    //
    // // 更新地图状态
    // const auto aligned_position_shift = getPositionShiftFromIndexShift(index_shift, resolution_);
    // origin_ += aligned_position_shift;
    // start_index_ = wrapIndexToRange(start_index_ + index_shift, size_); // 限制循环缓冲区起始索引在缓冲区范围内
    //
    // // 最复杂的是，要返回唯一的被删掉的grid，和新增的grid
    // // 如果只在一个维度上移动，那还算简单
    // // 如果在两个维度上移动，要考虑重合区域
    // // 如果在三个维度上移动，要考虑重合区域
    //
    // // 如果没有设置keep_remove，删除删掉栅格的数据
    //
    return true;
}

template <typename T>
void GridMap<T>::setOrigin(const Position& origin) {
    origin_ = origin;
}

template <typename T>
Index GridMap<T>::getIndex(const Position& position) const {
    return getIndexFromPosition(position, resolution_, origin_);
}

template <typename T>
Position GridMap<T>::getPosition(const Index& index) const {
    return getPositionFromIndex(index, resolution_, origin_);
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

}

#endif  //GRID_MAP_IMPL_HPP
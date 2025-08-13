// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef GRID_MAP_IMPL_HPP
#define GRID_MAP_IMPL_HPP

#include <cfloat>  // 提供 DBL_MAX
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
bool GridMap<T>::rayCast(const Position& end, std::vector<Index>& indices) const {

    indices.clear();

    auto origin_index = Index::Zero();
    auto end_index = getIndex(end);

    // 光线超出地图范围，结束
    if (!checkIfIndexValid(end_index, size_)) {
        return false; // TODO: 光线超出范围时，是否应该找将整条光线上的栅格记为free?即，过远的传感器数据是否可信？
    }

    // 光线终点为原点，结束
 //   if (origin_index == end_index) {
 //       return true;
 //   }

    // TODO:要不要先把原点放进来


    // 初始化
    Vector direction (end - origin_);
 //   auto length = direction.norm();
 //   direction = direction/length;   对方向向量归一化会引入无理数可能导致浮点数精度造成误差
    Index current_voxel = (origin_ / resolution_).cast<int>();
    Index last_voxel = (end / resolution_).cast<int>();

    Vector tMax, tDelta;
    Eigen::Vector3i step;

    for (int i = 0; i < 3; ++i) {
        step[i] = sign(direction[i]);

        if (step[i] != 0) {
            tMax[i] = step[i] * resolution_ / direction[i];
            tDelta[i] = resolution_ / direction[i] * step[i];
        }
        else{
            tMax[i] = DBL_MAX ;
            tDelta[i] = DBL_MAX ;
        }
    }

    // TODO:需不需要对step为负值的情况做处理

    indices.push_back(current_voxel);
    while(last_voxel != current_voxel) {
        if (tMax.x() < tMax.y()) {
            if (tMax.x() < tMax.z()) {
                current_voxel.x() += step.x();;
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
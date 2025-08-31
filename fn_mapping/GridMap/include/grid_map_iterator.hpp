// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <stdexcept>
#include "grid_map_math.hpp"

namespace finenav_2d {

template <typename T>
class GridMap; // 前向声明

template <typename T>
class GridMapIterator {
public:

    using value_type = T;
    using pointer = T*;
    using reference = T&;
    using map_type = GridMap<T>;

    // 仅用于创建end迭代器
    GridMapIterator() : map_(nullptr), current_index_(Index::Zero()), half_size_(Size::Zero()), is_end_(true) {}

    // 初始化迭代器
    GridMapIterator(map_type* map, bool is_end = false) : map_(map), is_end_(is_end) {
        if (map_) {
            half_size_ = map_->getSize() / 2;
            if (!is_end) {
                // 开始迭代器从最小索引开始
                current_index_ = Index(-half_size_.x(), -half_size_.y(), -half_size_.z());
            } else {
                // 结束迭代器设置为超出范围的索引
                current_index_ = Index(half_size_.x() + 1, -half_size_.y(), -half_size_.z());
            }
        } else {
            current_index_ = Index::Zero();
            half_size_ = Size::Zero();
        }
    }

    // 解引用操作符
    reference operator*() const {
        if (!map_ || is_end_) {
            throw std::runtime_error("Dereferencing invalid iterator");
        }
        return map_->at(current_index_);
    }

    // 箭头操作符
    pointer operator->() const {
        if (!map_ || is_end_) {
            throw std::runtime_error("Dereferencing invalid iterator");
        }
        return &(map_->at(current_index_));
    }

    // 前缀递增操作符
    GridMapIterator& operator++() {
        if (!map_ || is_end_) {
            return *this;
        }

        // 按照z-y-x的顺序递增
        ++current_index_.z();
        if (current_index_.z() > half_size_.z()) {
            current_index_.z() = -half_size_.z();
            ++current_index_.y();
            if (current_index_.y() > half_size_.y()) {
                current_index_.y() = -half_size_.y();
                ++current_index_.x();
                if (current_index_.x() > half_size_.x()) {
                    // 到达末尾
                    is_end_ = true;
                }
            }
        }
        return *this;
    }

    // 后缀递增操作符
    GridMapIterator operator++(int) {
        GridMapIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    // 相等比较操作符
    bool operator==(const GridMapIterator& other) const {
        if (map_ != other.map_) return false;
        if (is_end_ && other.is_end_) return true;
        if (is_end_ != other.is_end_) return false;
        return current_index_ == other.current_index_;
    }

    // 不等比较操作符
    bool operator!=(const GridMapIterator& other) const {
        return !(*this == other);
    }

    // 获取当前索引
    const Index& getIndex() const {
        return current_index_;
    }

    // 获取当前位置（世界坐标）
    Position getPosition() const {
        if (!map_ || is_end_) {
            throw std::runtime_error("Getting position from invalid iterator");
        }
        return map_->getPosition(current_index_);
    }

protected:
    map_type* map_;
    Size half_size_;
    Index current_index_;
    bool is_end_;
};

} // namespace finenav_2d

// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <vector>
#include <span>

#include "map_attribute.hpp"

namespace finenav_2d {

class MapInterface {
public:
    using Ptr = std::shared_ptr<MapInterface>;

    MapInterface(const size_t& rows, const size_t& cols) : rows_(rows), cols_(cols) {}

    virtual ~MapInterface() = default;

    // // 通过AttributeField访问占用状态 - 兼容旧接口
    // bool isOccupied(const Index & index) const {
    //     try {
    //         // 转换Index到相对坐标
    //         size_t row = index.x() - min_idx_.x();
    //         size_t col = index.y() - min_idx_.y();
    //         size_t depth = index.z() - min_idx_.z();
    //         return attribute_field_map_.at("occupancy", row, col, depth) > 0.5f;
    //     } catch (const std::out_of_range&) {
    //         return false; // 越界认为未占用
    //     }
    // }
    //


    // 获取属性字段访问器
    AttributeMap<float>& getAttributeFields() { return attribute_field_map_; }
    const AttributeMap<float>& getAttributeFields() const { return attribute_field_map_; }

    const size_t& rows() const { return rows_; }
    const size_t& cols() const { return cols_; }


protected:
    AttributeMap<float> attribute_field_map_;
    size_t rows_, cols_; // TODO: 是否需要知道min_idx_
};

} // namespace finenav_2d

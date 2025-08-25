// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <vector>
#include <span>

#include "map_attribute.hpp"

namespace finenav_2d {

/**
 * @note 用于防止基类为模板
 */
class MapInterfaceBase {
public:
    virtual ~MapInterfaceBase() = default;
};

/**
 * @note 用户实际继承的接口类
 * @note 仅作为访问接口，不进行访问权限控制
 */
template <typename T>
class MapInterface : public MapInterfaceBase {
public:
    using Ptr = std::shared_ptr<MapInterface>;

    MapInterface(const size_t& rows, const size_t& cols) : rows_(rows), cols_(cols) {}

    virtual ~MapInterface() = default;

    virtual std::span<T> getMapDataAt(const size_t& rows, const size_t& cols) const = 0;

    AttributeMap<T>& attributeMap() { return attribute_field_map_; }
    const size_t& rows() const { return rows_; }
    const size_t& cols() const { return cols_; }


protected:
    AttributeMap<T> attribute_field_map_;

    size_t rows_, cols_; // TODO: 是否需要知道min_idx_
};

} // namespace finenav_2d

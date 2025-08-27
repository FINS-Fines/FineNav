// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include "type_defs.hpp"
#include "map_attribute.hpp"

namespace finenav_2d {

class MapInterface {
public:
    using Ptr = std::shared_ptr<MapInterface>;

    MapInterface(const Index & min_idx, const Index & max_idx,const double & resolution) : min_idx_(min_idx), max_idx_(max_idx),resolution_(resolution){
        size_ = max_idx - min_idx + Index::Ones();
        if ((max_idx.array() < min_idx.array()).any()) {
            throw std::invalid_argument("MapInterface: max_idx must be >= min_idx in all dimensions");
        }
    }

    virtual ~MapInterface() = default;

    virtual bool isOccupied(const Index & index) const = 0;

    virtual float getZheightat(const Index & index) const = 0;

    AttributeFieldMap<float>& getAttributeFields() { return attribute_field_map_; }
    const Size& getSize() const {return size_;}
    const Index& getMinIndex() const { return min_idx_; }
    const Index& getMaxIndex() const { return max_idx_; }
    const double& getResolution() const { return resolution_; }

protected:
    AttributeFieldMap<float> attribute_field_map_;
    Index min_idx_;
    Index max_idx_;
    Size size_;
    double resolution_; // 仅供派生类使用
};

} // namespace finenav_2d

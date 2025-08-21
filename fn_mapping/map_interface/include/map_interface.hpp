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

    virtual ~MapInterface() = default;

    virtual bool isOccupied(const Index & index) const = 0;

    MapAttributeFields<float>& getAttributeFields() const { return *attribute_fields_; }

    Size getSize() const {return size_;}
    Index getMinIndex() const { return min_idx_; }
    Index getMaxIndex() const { return max_idx_; }

protected:
    std::shared_ptr<MapAttributeFields<float>> attribute_fields_;
    Index min_idx_;
    Index max_idx_;
    Size size_;
};

} // namespace finenav_2d

// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <span>

namespace finenav_2d {

class TerrainDataView {
public:
    using Getter = std::function<std::span<double>(const size_t&, const size_t&)>;

    using Ptr = std::shared_ptr<TerrainDataView>;

    explicit TerrainDataView(const size_t& size_x, const size_t& size_y, const Getter &getter)
        : size_x_(size_x), size_y_(size_y) ,getter_(getter) {}

    std::span<double> operator()(const size_t& idx_x, const size_t& idx_y) const {
        return getter_(idx_x, idx_y);
    }

    const size_t& sizeX() const { return size_x_; }
    const size_t& sizeY() const { return size_y_; }

protected:
    size_t size_x_, size_y_;
    Getter getter_;
};

} // namespace finenav_2d

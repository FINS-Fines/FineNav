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

    explicit TerrainDataView(const size_t& rows, const size_t cols, const Getter &getter)
        : rows_(rows), cols_(cols) ,getter_(getter) {}

    std::span<double> getMapDataAt(const size_t& row, const size_t& col) const {
        return getter_(row, col);
    }

protected:
    size_t rows_, cols_;
    Getter getter_;
};

} // namespace finenav_2d

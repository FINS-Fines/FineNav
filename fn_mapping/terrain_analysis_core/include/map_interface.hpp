// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <span>

namespace finenav_2d {

/**
 * @note 用户实际继承的接口类
 * @note 仅作为访问接口，不进行访问权限控制
 */
class TerrainDataView {
public:
    using Ptr = std::shared_ptr<TerrainDataView>;

    TerrainDataView(const size_t& rows, const size_t& cols) : rows_(rows), cols_(cols) {}

    virtual ~TerrainDataView() = default;

    virtual std::span<double> getMapDataAt(const size_t& rows, const size_t& cols) const = 0;

    const size_t& rows() const { return rows_; }
    const size_t& cols() const { return cols_; }

protected:
    size_t rows_, cols_; // TODO: 是否需要知道min_idx_
};

} // namespace finenav_2d

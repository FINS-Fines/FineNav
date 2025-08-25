// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <stdexcept>

namespace finenav_2d {

// TODO: 搬到util中，设置util命名空间

template <typename T>
class VectorMatrix {
public:
    VectorMatrix(const size_t& rows, const size_t& cols) : rows_(rows), cols_(cols) {
        data_.resize(rows_ * cols_);
    }
    ~VectorMatrix() = default;

    /**
     * @brief 访问指定位置的Vector
     */
    std::vector<T>& at(const size_t& row, const size_t& col) {
        if (exists(row, col)) {
            return data_[flatten(row, col)];
        }
        throw std::out_of_range("Index out of bounds in Attribute layer");
    }

    const std::vector<T>& at(const size_t& row, const size_t& col) const{
        if (exists(row, col)) {
            return data_[flatten(row, col)];
        }
        throw std::out_of_range("Index out of bounds in Attribute layer");
    }

    /**
     * @brief 访问指定位置的Vector中的指定元素
     * @note 可能会访问越界，需要调用者自行保证depth不越界
     */
    T& at(const size_t& row, const size_t& col, const size_t& vector_idx) {
        return data_.at(row, col)[vector_idx];
    }

    const T& at(const size_t& row, const size_t& col, const size_t& vector_idx) const {
        return data_.at(row, col)[vector_idx];
    }

private:
    bool exists(const size_t& row, const size_t& col) const {
        return row < rows_ && col < cols_;
    }

    size_t flatten(const size_t& row, const size_t& col)  const {
        return row * cols_ + col;
    }

    std::vector<std::vector<T>> data_;
    size_t rows_, cols_;

};

} // namespace finenav_2d


// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>
#include "vector_matrix.hpp"

namespace finenav_2d {

template <typename T>
class AttributeMap {
public:
    AttributeMap() = default;
    ~AttributeMap() = default;

    bool exists(const std::string& name) const {
        return field_map_.contains(name);
    }

    VectorMatrix<T>& at(const std::string& name) {
        if (exists(name)) {
            return field_map_.at(name);
        }
        throw std::out_of_range("Attribute does not exist");
    }

    const VectorMatrix<T>& at(const std::string& name) const {
        if (exists(name)) {
            return field_map_.at(name);
        }
        throw std::out_of_range("Attribute does not exist");
    }

    bool addAttributeField(const std::string& name, const size_t& row, const size_t& col, const T& default_value) {
        if (exists(name)) { return false; }
        field_map_.insert({name, VectorMatrix<T>(row, col)});
        return true;
    }

    bool removeAttributeField(const std::string& name) {
        if (!exists(name)) { return false; }
        field_map_.erase(name);
        return true;
    }

private:
    std::unordered_map<std::string, VectorMatrix<T>> field_map_;
};

} // namespace finenav_2d

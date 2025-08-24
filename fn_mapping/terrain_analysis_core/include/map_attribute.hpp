// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>
#include "type_defs.hpp"

namespace finenav_2d {

template <typename DataT>
class AttributeFiled {
public:
    AttributeFiled(const Index& min_idx, const Index& max_idx, const DataT& default_value)
        : min_idx_(min_idx), max_idx_(max_idx) {
        size_ = max_idx - min_idx + Index::Ones();
        data_.resize(size_.x() * size_.y() * size_.z(), default_value);
    }
    ~AttributeFiled() = default;

    bool exists(const Index& idx) const {
        return (idx.array() >= min_idx_.array()).all() && (idx.array() <= max_idx_.array()).all();
    }

    DataT& at (const Index& idx) {
        if (exists(idx)) {
            return data_[flatten(idx)];
        }
        throw std::out_of_range("Index out of bounds in Attribute layer");
    }

    DataT at(const Index& idx) const {
        if (exists(idx)) {
            return data_[flatten(idx)];
        }
        throw std::out_of_range("Index out of bounds in Attribute layer");
    }

private:
     int flatten(const Index & idx) const {
        return (idx.x() - min_idx_.x()) * size_.y() * size_.z() +
               (idx.y() - min_idx_.y()) * size_.z() +
               (idx.z() - min_idx_.z());
    }

    std::vector<DataT> data_;
    Index min_idx_;
    Index max_idx_;
    Size size_;
};

template <typename DataT>
class AttributeFieldMap {
public:
    AttributeFieldMap() = default;
    ~AttributeFieldMap() = default;

    bool exists(const std::string& name) const {
        return field_map.find(name) != field_map.end();
    }

    DataT& at(const std::string& name, const Index& idx) {
        if (exists(name)) {
            return field_map.at(name).at(idx);
        }
        throw std::out_of_range("Attribute layer does not exist");
    }

    DataT at(const std::string& name, const Index& idx) const {
        if (exists(name)) {
            return field_map.at(name).at(idx);
        }
        throw std::out_of_range("Attribute layer does not exist");
    }

    bool addAttributeField(const std::string& name, const Index& min_idx, const Index& max_idx, const DataT& default_value) {
        if ((max_idx.array() < min_idx.array()).any()) {
            throw std::invalid_argument("AttributeFieldMap: max_idx must be >= min_idx in all dimensions");
        }

        if (exists(name)) { return false; }
        field_map.insert({name, AttributeFiled<DataT>(min_idx, max_idx, default_value)});
        return true;
    }

    bool removeAttributeField(const std::string& name) {
        if (!exists(name)) { return false; }
        field_map.erase(name);
        return true;
    }

private:
    std::unordered_map<std::string, AttributeFiled<DataT>> field_map;
};

} // namespace finenav_2d

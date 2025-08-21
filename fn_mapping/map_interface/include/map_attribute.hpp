// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>
#include "type_defs.hpp"

namespace finenav_2d {

template <typename DataT>
class MapAttribute {
public:
    MapAttribute(const Index& min_idx, const Index& max_idx, const DataT& default_value)
        : min_idx_(min_idx), max_idx_(max_idx) {
        size_ = max_idx - min_idx + Index::Ones();
        data_.resize(size_.x() * size_.y() * size_.z(), default_value);
    }

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
class MapAttributeFields {
public:

    bool exists(const std::string& name) const {
        return attribute_fields_.find(name) != attribute_fields_.end();
    }

    DataT& at(const std::string& name, const Index& idx) {
        if (exists(name)) {
            return attribute_fields_.at(name).at(idx);
        }
        throw std::out_of_range("Attribute layer does not exist");
    }

    DataT at(const std::string& name, const Index& idx) const {
        if (exists(name)) {
            return attribute_fields_.at(name).at(idx);
        }
        throw std::out_of_range("Attribute layer does not exist");
    }

    bool addAttributeLayer(const std::string& name, const Index& min_idx, const Index& max_idx, const DataT& default_value) {
        if (exists(name)) { return false; }
        attribute_fields_[name] = MapAttribute<DataT>(min_idx, max_idx, default_value);
        return true;
    }

    bool removeAttributeLayer(const std::string& name) {
        if (!exists(name)) { return false; }
        attribute_fields_.erase(name);
        return true;
    }

private:
    std::unordered_map<std::string, MapAttribute<DataT>> attribute_fields_;
};

} // namespace finenav_2d

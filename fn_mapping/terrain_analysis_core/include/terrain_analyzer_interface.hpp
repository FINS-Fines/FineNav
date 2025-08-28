// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <span>

namespace finenav_2d {

class TerrainAnalyzerInterface {
public:
    using Ptr = std::shared_ptr<TerrainAnalyzerInterface>;
    using Getter = std::function<std::span<float>(const size_t&, const size_t&)>;
    using Filter = std::function<bool(const float&)>;
    using Setter = std::function<void(const size_t&, const size_t&, const float&)>;


    /**
     * @note 原点定义在terrainMap的左下角
     */
    explicit TerrainAnalyzerInterface( const size_t& size_x, const size_t& size_y,
        const float& origin_x, const float& origin_y,
        const Getter& getter, const Setter& setter, const Filter& filter = nullptr)
        : size_x_(size_x), size_y_(size_y) ,origin_x_(origin_x), origin_y_(origin_y), getter_(getter), filter_(filter), setter_(setter) {

        // 默认Filter滤除NAN
        if (filter_ == nullptr) {
            filter_ = [](const float& value) -> bool {
                return !std::isnan(value);
            };
        }
    }

    std::span<float> getMap(const size_t& idx_x, const size_t& idx_y) const {
        return getter_(idx_x, idx_y);
    }

    bool dataIsValid(const float& value) const {
        return filter_(value);
    }

    void setResult(const size_t& idx_x, const size_t& idx_y, const float& value) const {
        return setter_(idx_x, idx_y, value);
    }

    const size_t& sizeX() const { return size_x_; }
    const size_t& sizeY() const { return size_y_; }

protected:
    size_t size_x_, size_y_;
    float origin_x_, origin_y_;
    Getter getter_;
    Filter filter_;
    Setter setter_;
};

} // namespace finenav_2d

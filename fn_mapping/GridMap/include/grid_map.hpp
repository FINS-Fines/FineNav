// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <span>

#include "grid_map_math.hpp"
#include "grid_map_iterator.hpp"

namespace finenav_2d {

/**
 * @brief 支持滚动更新的三维栅格地图类
 * @details 该地图维护两种索引方式，(1) Index 相对于地图原点的索引; (2) Position 相对于世界坐标系的坐标
 * @details 地图内部通过维护地图原点在世界坐标系中的位置，实现地图的滚动更新
 * @note 该栅格地图的原点总是在地图中心
 * @note 不支持地图原点的姿态变化
 */
template <typename T>
class GridMap {
public:
    /**
     * @brief 构造函数
     * @param length 栅格地图的尺寸，单位为米
     * @param resolution 栅格地图的分辨率，单位为米
     * @param origin 栅格地图的原点坐标
     * @note 所有地图数据均为零初始化
     */
    GridMap(const Length& length, const double& resolution, const Position& origin = Position::Zero());

    /**
     * @brief 默认的构造函数
     */
    GridMap(const GridMap&) = default;
    GridMap& operator=(const GridMap&) = default;
    GridMap(GridMap&&) = default;
    GridMap& operator=(GridMap&&) = default;

    /**
     * @brief 析构函数
     */
    virtual ~GridMap() = default;

    /**
     * @brief 访问某个位置的栅格数据
     * @param position 世界坐标系下的位置
     * @return 对应位置的栅格数据
     * @throw
     */
    T& atPosition(const Position& position);

    /**
     * @brief 访问某个坐标点的栅格数据（常量版本）
     * @param position 世界坐标系下的位置
     * @return 对应位置的栅格数据
     */
    T atPosition(const Position& position) const;

    /**
     * @brief 访问某个index的栅格数据
     * @param index 栅格地图的索引
     * @return 对应的栅格数据
     */
    T& at(const Index& index);

    /**
     * @brief 访问某个index的栅格数据（常量版本）
     * @param index 栅格地图的索引
     * @return 对应的栅格数据
     */
    T at(const Index& index) const;


    /**
     * @brief 访问某个(x,y)索引上的一串栅格数据
     * @param x x索引
     * @param y y索引
     * @return 一串栅格数据的span
     * @note 该函数的实现基于内部缓冲区z-y-x的存储顺序
     */
    std::span<T> getVoxelsAlongZ(const int& x, const int& y);


    /**
     * @brief 访问某个(x,y)索引上的一串栅格数据（常量版本）
     * @param x x索引
     * @param y y索引
     * @return 一串栅格数据的span
     * @note 该函数的实现基于内部缓冲区z-y-x的存储顺序
     */
    std::span<const T> getVoxelsAlongZ(const int& x, const int& y) const;

    /**
     * @brief 清空地图数据
     */
    void clear();

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param position  地图原点移动到的新的位置
     * @return 如果地图实际发生移动返回true，否则返回false
     * @note 删除的栅格数据会被清除
     */
    bool moveTo(const Position& position);

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param[in] position 地图原点移动到的新的位置
     * @param[in] keep_removed 是否保留被删除的栅格数据
     * @param[out] removed_region 移动过程中被删除的位置
     * @return 如果地图实际发生移动返回true，否则返回false
     * @note 当再次调用moveTo()时，之前返回的indices没有意义
     */
    bool moveTo(const Position& position, const bool keep_removed, std::vector<Position>& removed_region);

    /**
     * @brief 光线投射
     * @param origin 光线的终点位置
     * @param end 光线的终点位置
     * @param indices 返回光线经过的栅格索引
     * @return 如果光线终点超出地图范围，返回false；否则返回true
     */
    bool rayCast(const Position& origin,const Position& end, std::vector<Index>& indices) const;

    /**
     * @brief 设置地图的原点，即地图在世界坐标系中的位置
     * @param origin 地图的原点坐标
     */
    void setOrigin(const Position& origin);

    /**
     * @brief 获取满足某个条件的栅格索引
     * @param[out] indices 输出满足条件的栅格索引
     * @param[in] condition 条件函数，接受栅格数据类型T的引用，返回bool值
     */
    void selectCellsByCondition(std::vector<Index>& indices, std::function<bool(const T&)> condition) const;

    /**
     * @brief 获取地图中某个位置的索引
     * @param position 世界坐标系下的位置
     * @return 对应的栅格地图索引
     */
    Index getIndex(const Position& position) const;

    /**
     * @brief 获取地图中某个索引对应的栅格中心位置
     * @param index 栅格地图的索引
     * @return 对应的栅格中心位置
     */
    Position getPosition(const Index& index) const;

    /**
     * @brief 获取地图的尺寸
     * @return 地图的尺寸，单位为米
     */
    Length getLength() const;

    /**
     * @brief 获取栅格地图的大小
     * @return 栅格地图的大小，单位为栅格数
     */
    Size getSize() const;

    /**
     * @brief 获取地图分辨率
     * @return 地图分辨率，单位为米
     */
    double getResolution() const;

    /**
     * @brief 获取地图原点
     * @return 地图原点坐标
     */
    Position getOrigin() const;

    /**
     * @brief 地图边界框的最小索引
     * @return min_index
     */
    Index getMinIndex() const;


    /**
     * @brief 地图边界框的最大索引
     * @return max_index
     */
    Index getMaxIndex() const;

    /**
     * @brief 检查某个位置是否在地图范围内
     * @param position 世界坐标系下的位置
     * @return 如果位置在地图范围内返回true，否则返回false
     */
    bool isInside(const Position& position) const;

    /**
     * @brief 检查某个位置是否在地图范围内
     * @param index 栅格地图的索引
     * @return 如果位置在地图范围内返回true，否则返回false
     */
    bool isInside(const Index& index) const;


    /// @brief 迭代器接口
    using iterator = GridMapIterator<T>;
    using const_iterator = GridMapIterator<const T>;

    iterator begin() { return iterator(this, false); }
    iterator end() { return iterator(this, true); }
    const_iterator begin() const { return const_iterator(this, false); }
    const_iterator end() const { return const_iterator(this, true); }

private:
    std::vector<T> data_; // 存储所有栅格数据
    Length length_;       // 栅格地图的长度，单位为米
    Index start_index_;   // 地图数据的起始索引
    Size size_;           // 栅格地图的尺寸，一定为奇数，以保证原点在中心位置
    double resolution_;   // 栅格地图的分辨率，单位为米
    Position origin_;     // 当前栅格地图的原点，用于定义世界坐标系下地图的位置

    double inv_resolution_;
    Size half_size_;

};

} // namespace finenav_2d

#include "grid_map_impl.hpp"

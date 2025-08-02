// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_GRID_MAP_3D_HPP
#define FINENAV2D_GRID_MAP_3D_HPP

#include <vector>
#include "grid_map_math.hpp"

namespace finenav_2d {

/**
 * @brief 支持滚动更新的三维栅格地图类
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
     * @param position 相对于地图原点的位置
     * @return 对应位置的栅格数据
     * @throw
     */
    T& atPosition(const Position& position);

    /**
     * @brief 访问某个坐标点的栅格数据（常量版本）
     * @param position 相对于地图原点的坐标点
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
     * @brief 清空地图数据
     */
    void clear();

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param position  地图原点移动到的新的位置
     * @return 如果移动成功返回true，否则返回false
     * @note 删除的栅格数据会被清除
     */
    bool moveTo(const Position& position);

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param[in] position 地图原点移动到的新的位置
     * @param[in] keep_removed 是否保留被删除的栅格数据
     * @param[out] indices 移动过程中被删除的栅格索引
     * @return 如果地图实际发生移动返回true，否则返回false
     * @note 当再次调用moveTo()时，之前返回的indices没有意义
     */
    bool moveTo(const Position& position, bool keep_removed, std::vector<Index>& indices);

    /**
     * @brief 设置地图的原点，即地图在世界坐标系中的位置
     * @param origin 地图的原点坐标
     */
    void setOrigin(const Position& origin);

    /**
     * @brief 获取地图中某个位置的索引
     * @param position 相对于地图原点的位置
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

private:
    std::vector<T> data_; // 存储所有栅格数据
    Length length_;       // 栅格地图的长度，单位为米
    Index start_index_;   // 地图数据的起始索引
    Size size_;           // 栅格地图的尺寸，一定为奇数，以保证原点在中心位置
    double resolution_;   // 栅格地图的分辨率，单位为米
    Position origin_;     // 当前栅格地图的原点，用于定义世界坐标系下地图的位置

};

} // namespace finenav_2d

#include "grid_map_impl.hpp"
#endif  //FINENAV2D_GRID_MAP_3D_HPP
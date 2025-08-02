// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef GRID_MAP_MATH_HPP
#define GRID_MAP_MATH_HPP

#include <Eigen/Core>

namespace finenav_2d {
using Index = Eigen::Vector3i;
using Size = Eigen::Vector3i;
using Position = Eigen::Vector3d;
using Length = Eigen::Vector3d;
using Vector = Eigen::Vector3d;

/**
 * @brief 将索引向量的每个元素取模，确保索引在指定的范围内
 * @param index 输入的索引向量
 * @param size 指定的范围大小
 * @return 取模后的索引向量
 * @note 输入的索引向量可以是负数元素
 */
inline Index wrapIndexToRange(const Index& index, const Size& size) {
    Index result;
    for (int i = 0; i < index.size(); ++i) {
        result[i] = (index[i] + size[i]) % size[i];
    }
    return result;
}

/**
 * @brief 将位置偏移转换为索引偏移
 * @param position_shift 位置偏移量
 * @param resolution 栅格地图的分辨率
 * @return 对应的索引偏移量
 */
inline Index getIndexShiftFromPositionShift(const Position& position_shift, const double& resolution) {
    Vector index_shift_vector = position_shift / resolution;
    return index_shift_vector.array().round().matrix().cast<int>();

}

/**
 * @brief 将索引偏移转换为位置偏移
 * @param index_shift 索引偏移量
 * @param resolution 栅格地图的分辨率
 * @return 对应的位移偏移量
 */
inline Position getPositionShiftFromIndexShift(const Index& index_shift, const double& resolution) {
    return index_shift.cast<double>() * resolution;
}


/**
 * @brief 将世界坐标系下的位置转换为栅格地图索引
 * @param position 访问的世界坐标系下的位置
 * @param resolution 栅格地图的分辨率
 * @param origin 栅格地图的原点位置
 * @return 对应的栅格地图索引
 */
inline Index getIndexFromPosition(const Position& position, const double& resolution, const Position& origin) {
    return getIndexShiftFromPositionShift(position - origin, resolution);
}

/**
 * @brief 将栅格地图索引转换为世界坐标系下的位置
 * @param[in] index 栅格地图索引
 * @param[in] resolution 栅格地图的分辨率
 * @param[in] origin 栅格地图的原点位置
 * @return 对应的栅格中心在世界坐标系下的位置
 */
inline Position getPositionFromIndex(const Index& index, const double& resolution, const Position& origin) {
    return origin + getPositionShiftFromIndexShift(index, resolution);
}

/**
 * @brief 将地图索引转换为缓冲区索引
 * @param index 地图索引
 * @param buffer_size 缓冲区中地图的大小
 * @param buffer_start_index 循环缓冲区的起始索引
 * @return 对应的缓冲区索引
 */
inline Index getBufferIndex(const Index& index,
                            const Size& buffer_size,
                            const Index& buffer_start_index = Index::Zero()) {
    const Index unwrapped_index = buffer_size / 2 + index;
    return wrapIndexToRange(unwrapped_index - buffer_start_index, buffer_size);
}

/**
 * @brief 检查索引是否在栅格地图的有效范围内
 * @param index 栅格地图索引
 * @param size 栅格地图的大小
 * @return 如果索引在范围内返回true，否则返回false
 */
inline bool checkIfIndexValid(const Index& index, const Size& size) {
    return (index.cwiseAbs().array() <= size.array() / 2).all();
}

/**
 * @brief 对于两个栅格地图A和B，计算差集A-B
 * @param[in] index_shift 相对于地图A的索引偏移量
 * @param[in] size 两个栅格地图的大小
 * @param[out] difference_indices 输出的差集索引
 * @note 索引定义在地图A的坐标系下，栅格地图的原点位于地图中心
 */
inline void getDifference(const Index& index_shift, const Size& size, std::vector<Index>& difference_indices) {}


} // namespace finenav_2d

#endif  //GRID_MAP_MATH_HPP
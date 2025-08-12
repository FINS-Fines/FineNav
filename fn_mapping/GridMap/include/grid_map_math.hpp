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
 * @param index 输入的索引
 * @param size 指定的范围大小
 * @return 取模后的索引向量
 * @note 输入的索引向量可以是负数元素
 * TODO: 是否能够优化expensive的取模运算
 */
inline int wrapIndexToRange(const int& index, const int& size) {
    // 通过分支减少高昂的取模运算
    if (index < size) {
        if (index >= 0) { // [0, size)
            return index;
        } else if (index >= -size) { // [-size, 0)
            return index + size;
        } else { // [-inf, -size)
            return index % size + size;
        }
    } else if (index < size*2) { // [size, 2*size)
        return index - size;
    } else { // [2*size, inf)
        return index % size;
    }
}

inline Index wrapIndexToRange(const Index& index, const Size& size) {
    Index result;
    for (int i = 0; i < index.size(); i++) {
        result[i] = wrapIndexToRange(index[i], size[i]);
    }
    return result;
}

/**
 * @brief 返回一个数的符号
 * @Tparam 数据类型，支持 >, < 操作符
 * @param x 输入的数值
 * @return -1 如果 x < 0, -1 如果 x == 0, 0 如果 x > 0, 1
 */
template <typename T>
inline int sign(T x) {
    return (x > 0) - (x < 0);
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
 * @note 按照x-y-z的顺序在缓冲区存储
 */
inline int getBufferIndex(const Index& index,
                            const Size& buffer_size,
                            const Index& buffer_start_index = Index::Zero()) {
    const Index unwrapped_index = buffer_size / 2 + index;
    const Index wrapped_index = wrapIndexToRange(unwrapped_index - buffer_start_index, buffer_size);
    return wrapped_index.x() + wrapped_index.y() * buffer_size.x() + wrapped_index.z() * buffer_size.x() * buffer_size.y();

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
 * @param[in] size 两个栅格地图的大小，地图大小需要为奇数
 * @param[out] difference_indices 输出的差集索引
 * @note 索引定义在地图A的坐标系下，栅格地图的原点位于地图中心
 */
inline void getDifferenceSet(const Index& index_shift, const Size& size, std::vector<Index>& difference_indices) {
    difference_indices.clear();

    Index half_size = size / 2;
    Index a_min = -half_size;
    Index a_max = half_size;

    Index b_min = -half_size + index_shift;
    Index b_max = half_size + index_shift;
    
    // 如果B完全在A外面，直接返回整个A
    if ((b_min.array() > half_size.array()).any() || (b_max.array() < -half_size.array()).any()) {
        for (int x = -half_size.x(); x <= half_size.x(); ++x) {
            for (int y = -half_size.y(); y <= half_size.y(); ++y) {
                for (int z = -half_size.z(); z <= half_size.z(); ++z) {
                    difference_indices.emplace_back(x, y, z);
                }
            }
        }
        return;
    }

    // 辅助函数：添加指定范围的索引到差集
    auto add_range = [&](int x_start, int x_end, int y_start, int y_end, int z_start, int z_end) {
        // 限制范围在 A 内
        x_start = std::max(x_start, a_min.x());
        x_end = std::min(x_end, a_max.x());
        y_start = std::max(y_start, a_min.y());
        y_end = std::min(y_end, a_max.y());
        z_start = std::max(z_start, a_min.z());
        z_end = std::min(z_end, a_max.z());

        if (x_start > x_end || y_start > y_end || z_start > z_end) return;

        for (int x = x_start; x <= x_end; ++x) {
            for (int y = y_start; y <= y_end; ++y) {
                for (int z = z_start; z <= z_end; ++z) {
                    difference_indices.emplace_back(x, y, z);
                }
            }
        }
    };

    // 依次判断 z,x,y
    if (a_min.z() <= b_max.z() && b_max.z() <= a_max.z()) {
        add_range(a_min.x(), a_max.x(), a_min.y(), a_max.y(), b_max.z() + 1, a_max.z());

        // x
        if (a_min.x() <= b_min.x() && b_min.x() <= a_max.x()) {
            add_range(a_min.x(), b_min.x()-1, a_min.y(), a_max.y(), a_min.z(), b_max.z());
            
            // y
            if (a_min.y() <= b_min.y() && b_min.y() <= a_max.y()) {
                add_range(b_min.x(), b_max.x(), a_min.y(), b_min.y() - 1, a_min.z(), b_max.z());
            } else if(a_min.y() <= b_max.y() && b_max.y() <= a_max.y()){
                add_range(b_min.x(), b_max.x(), b_max.y() + 1, a_max.y(), a_min.z(), b_max.z());
            }
        } else if(a_min.x() <= b_max.x() && b_max.x() <= a_max.x()){
            add_range(b_max.x()+1, a_max.x(), a_min.y(), a_max.y(), a_min.z(), b_max.z());
            
            // y
            if (a_min.y() <= b_min.y() && b_min.y() <= a_max.y()) {
                add_range(b_min.x(), b_max.x(), a_min.y(), b_min.y() - 1, a_min.z(), b_max.z());
            } else if(a_min.y() <= b_max.y() && b_max.y() <= a_max.y()){
                add_range(b_min.x(), b_max.x(), b_max.y() + 1, a_max.y(), a_min.z(), b_max.z());
            }
        }
    } else if(a_min.z() <= b_min.z() && b_min.z() <= a_max.z()){
        add_range(a_min.x(), a_max.x(), a_min.y(), a_max.y(), a_min.z(), b_min.z() - 1);

        // x
        if (a_min.x() <= b_min.x() && b_min.x() <= a_max.x()) {
            add_range(a_min.x(), b_min.x()-1, a_min.y(), a_max.y(), b_min.z(), a_max.z());
            
            if (a_min.y() <= b_min.y() && b_min.y() <= a_max.y()) {
                add_range(b_min.x(), b_max.x(), a_min.y(), b_min.y() - 1, b_min.z(), a_max.z());
            } else if(a_min.y() <= b_max.y() && b_max.y() <= a_max.y()){
                add_range(b_min.x(), b_max.x(), b_max.y() + 1, a_max.y(), b_min.z(), a_max.z());
            }
        } else if (a_min.x() <= b_max.x() && b_max.x() <= a_max.x()){
            add_range(b_max.x()+1, a_max.x(), a_min.y(), a_max.y(), b_min.z(), a_max.z());
            
            if (a_min.y() <= b_min.y() && b_min.y() <= a_max.y()) {
                add_range(b_min.x(), b_max.x(), a_min.y(), b_min.y() - 1, b_min.z(), a_max.z());
            } else if(a_min.y() <= b_max.y() && b_max.y() <= a_max.y()){
                add_range(b_min.x(), b_max.x(), b_max.y() + 1, a_max.y(), b_min.z(), a_max.z());
            }
        }
    }
}


} // namespace finenav_2d

#endif  //GRID_MAP_MATH_HPP
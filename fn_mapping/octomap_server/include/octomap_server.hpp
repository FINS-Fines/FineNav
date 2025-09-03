// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <octomap/octomap.h>

namespace finenav_2d {

class OctoMapServer {
public:
    using OcTreeT = octomap::OcTree;
    using Point = octomap::point3d;

    struct BoundingRegion {
        Point min, max;
        bool valid;

        BoundingRegion() : valid(false) {}
        BoundingRegion(const Point& minPt, const Point& maxPt)
            : min(minPt), max(maxPt), valid(true) {}
    };

    /**
     * @brief 移动差异区域的遍历模式
     */
    enum class MoveDifferenceMode {
        REMOVED,  ///< 只遍历移动前有、移动后没有的区域
        ADDED,    ///< 只遍历移动前没有、移动后有的区域
        BOTH      ///< 同时遍历REMOVED和ADDED区域
    };

    OctoMapServer(const double& res): octree_(res) {}

    /**
      * @brief 构造函数，初始化八叉树
      * @param resolution 八叉树分辨率
      */

    /**
     * @brief 遍历移动差异区域 - 公共接口
     * @param original_min 原始边界框的最小点
     * @param original_max 原始边界框的最大点
     * @param moved_distance 移动向量
     * @param mode 遍历模式：REMOVED, ADDED, 或 BOTH
     * @param callback 对每个遍历到的叶子节点调用的回调函数
     */
    void traverseMoveDifferenceRegion(
        const Point& original_min,
        const Point& original_max,
        const Point& moved_distance,
        const std::function<void(OcTreeT::leaf_bbx_iterator&)> &callback,
        MoveDifferenceMode mode = MoveDifferenceMode::REMOVED);

    /**
     * @brief 获取八叉树对象
     */
    OcTreeT& getOctree() { return octree_; }

private:
    /**
     * @brief 遍历移动差异区域，返回Removed区域
     * @param original_min 原始边界框的最小点
     * @param original_max 原始边界框的最大点
     * @param moved_distance 移动向量
     * @param callback 对每个遍历到的叶子节点调用的回调函数
     */
    void traverseMoveDifferenceRegionImpl(
        const Point& original_min,
        const Point& original_max,
        const Point& moved_distance,
        const std::function<void(OcTreeT::leaf_bbx_iterator&)> &callback);

    /**
     * @brief 将移动差异区域划分为最多3个轴向的长方体区域
     * @param original_min 原始边界框的最小点
     * @param original_max 原始边界框的最大点
     * @param moved_distance 移动向量
     * @return 包含差异区域的长方体区域列表
     */
    std::vector<BoundingRegion> calculateMoveDifferenceRegions(
        const Point& original_min,
        const Point& original_max,
        const Point& moved_distance);

    OcTreeT octree_;

};


}

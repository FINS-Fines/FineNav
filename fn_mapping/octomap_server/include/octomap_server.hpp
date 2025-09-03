// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <functional>
#include <octomap/octomap.h>

namespace finenav_2d {

class OctoMapServer {
public:
    using OcTreeT = octomap::OcTree;
    using Point = octomap::point3d;
    using TraverseCallback = std::function<void(OcTreeT::leaf_bbx_iterator&, const Point&)>;

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
     * @brief 遍历移动差异区域 - 公共接口
     * @param original_min 原始边界框的最小点
     * @param original_max 原始边界框的最大点
     * @param moved_distance 移动向量
     * @param callback 对每个遍历到的叶子节点调用的回调函数，参数1: 叶子节点迭代器， 参数2：此时对应的体素坐标
     * @param expand_to_max_depth 是否展开到最大深度，在不改变八叉树结构前提下，将不到max_depth的叶子节点展开
     * @param mode 遍历模式：REMOVED, ADDED, 或 BOTH
     * @note 如果不展开到最大深度，callback参数2就是参数1迭代器的中心点坐标；如果展开，则没有此对应关系
     */
    void traverseMoveDifferenceRegion(
        const Point& original_min,
        const Point& original_max,
        const Point& moved_distance,
        const TraverseCallback& callback,
        bool expand_to_max_depth = false,
        MoveDifferenceMode mode = MoveDifferenceMode::REMOVED
        );

    /**
     * @brief 获取八叉树
     */
    OcTreeT& getOctree() { return octree_; }

private:
    /**
     * @brief 遍历移动差异区域，返回Removed区域
     * @param original_min 原始边界框的最小点
     * @param original_max 原始边界框的最大点
     * @param moved_distance 移动向量
     * @param callback 对每个遍历到的叶子节点调用的回调函数
     * @param expand_to_max_depth 是否展开到最大深度，在不改变八叉树结构前提下，将不到max_depth的叶子节点展开
     */
    void traverseMoveDifferenceRegionImpl(
        const Point& original_min,
        const Point& original_max,
        const Point& moved_distance,
        const TraverseCallback& callback,
        bool expand_to_max_depth);

    /**
     * @brief 遍历移动差异区域，返回Removed区域
     * @param original_min 原始边界框的最小点
     * @param original_max 原始边界框的最大点
     * @param moved_distance 移动向量
     * @param it 八叉树叶子节点边界框迭代器
     * @param callback 对每个遍历到的叶子节点调用的回调函数
     */
    void expandToMaxDepth(
        const Point& original_min,
        const Point& original_max,
        const Point& moved_distance,
        OcTreeT::leaf_bbx_iterator& it,
        const TraverseCallback& callback);

    /**
     * @brief 检查一个点是否在移动差异的Removed区域内
     * @param original_min 原始边界框的最小点
     * @param original_max 原始边界框的最大点
     * @param moved_distance 移动向量
     * @param point 要检查的点
     * @return 如果点在Removed区域内返回true，否则返回false
     */
    bool isInMoveDifferenceRegion(
        const Point& original_min,
        const Point& original_max,
        const Point& moved_distance,
        const Point& point);

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

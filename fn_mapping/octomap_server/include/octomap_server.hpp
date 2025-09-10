// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <functional>
#include <octomap/octomap.h>
#include <octomap/HeightOcTree.h>


namespace finenav_2d {

class OctoMapServer {
public:
    using OcTreeT = octomap::HeightOcTree;
    using Point = octomap::point3d;
    using IteratorBase = octomap::OcTreeBaseImpl<octomap::OcTreeNodeHeight,octomap::AbstractOccupancyOcTree>::iterator_base<octomap::OcTreeNodeHeight>;
    using TraverseCallback = std::function<void(IteratorBase*)>;

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
     * 读取八叉树文件
     * @param filename 文件路径
     * @return 成功返回true，失败返回false
     * @note 支持.ot格式
     */
    bool openFile(const std::string& filename);

    /**
     * @brief 遍历移动差异区域 - 公共接口
     * @param original_min 原始边界框的最小点
     * @param original_max 原始边界框的最大点
     * @param moved_distance 移动向量
     * @param callback 对每个遍历到的叶子节点调用的回调函数，参数1: 叶子节点迭代器基类指针， 参数2：此时对应的体素坐标
     * @param expand_to_max_depth 是否展开到最大深度。为true时使用expand_leaf_bbx_iterator自动将非最大深度叶子节点展开到最大深度；为false时使用普通leaf_bbx_iterator
     * @param mode 遍历模式：REMOVED, ADDED, 或 BOTH
     * @note expand_to_max_depth=true时，会自动调用expandNode()修改八叉树结构，将遇到的非最大深度叶子节点展开为8个子节点
     * @note 回调函数的第一个参数是迭代器基类指针，用户可以通过dynamic_cast转换为具体的迭代器类型以使用多态特性
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
     * @brief 遍历移动差异区域的内部实现，返回Removed区域
     * @param original_min 原始边界框的最小点
     * @param original_max 原始边界框的最大点
     * @param moved_distance 移动向量
     * @param callback 对每个遍历到的叶子节点调用的回调函数
     * @param expand_to_max_depth 为true时使用expand_leaf_bbx_iterator，为false时使用leaf_bbx_iterator
     * @note 根据expand_to_max_depth参数选择不同的迭代器：
     *       - false: 使用leaf_bbx_iterator，遍历现有的叶子节点
     *       - true: 使用expand_leaf_bbx_iterator，自动展开非最大深度叶子节点到最大深度
     */
    void traverseMoveDifferenceRegionImpl(
        const Point& original_min,
        const Point& original_max,
        const Point& moved_distance,
        const TraverseCallback& callback,
        bool expand_to_max_depth);

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

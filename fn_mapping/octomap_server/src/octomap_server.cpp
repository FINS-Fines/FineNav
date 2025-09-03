// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <queue>

#include "octomap_server.hpp"

#include <oneapi/tbb/detail/_template_helpers.h>

namespace finenav_2d {


void OctoMapServer::traverseMoveDifferenceRegion(
    const Point& original_min,
    const Point& original_max,
    const Point& moved_distance,
    const TraverseCallback& callback,
    bool expand_to_max_depth,
    MoveDifferenceMode mode) {

    switch (mode) {
        case MoveDifferenceMode::REMOVED:
            traverseMoveDifferenceRegionImpl(original_min, original_max, moved_distance, callback, expand_to_max_depth);
            break;

        case MoveDifferenceMode::ADDED:  // 将moved_distance取反，相当于反向移动
            traverseMoveDifferenceRegionImpl(original_min, original_max, -moved_distance, callback, expand_to_max_depth);
            break;

        case MoveDifferenceMode::BOTH:
            traverseMoveDifferenceRegionImpl(original_min, original_max, moved_distance, callback, expand_to_max_depth);
            traverseMoveDifferenceRegionImpl(original_min, original_max, -moved_distance, callback, expand_to_max_depth);
            break;

        default:
            throw std::invalid_argument("[OcotMapServer]: Invalid mode");
    }
}


void OctoMapServer::traverseMoveDifferenceRegionImpl(
    const Point& original_min,
    const Point& original_max,
    const Point& moved_distance,
    const TraverseCallback& callback,
    bool expand_to_max_depth) {

    // Calculate the difference regions
    std::vector<BoundingRegion> regions = calculateMoveDifferenceRegions(
        original_min, original_max, moved_distance);

    for (size_t i = 0; i < regions.size(); ++i) {
        const BoundingRegion& region = regions[i];
        if (!region.valid) continue;

        if (expand_to_max_depth) {
            // 使用expand_leaf_bbx_iterator，自动展开到最大深度,对处于L形区域内node的调用回调函数
            for (OcTreeT::expand_leaf_bbx_iterator it = octree_.begin_expand_leafs_bbx(region.min, region.max, 0),
                end = octree_.end_expand_leafs_bbx(); it != end; ++it) {
                if (isInMoveDifferenceRegion(original_min, original_max, moved_distance, it.getCoordinate())) {
                    callback(&it);
                }
            }
        } else {
            // 使用普通的leaf_bbx_iterator
            for (OcTreeT::leaf_bbx_iterator it = octree_.begin_leafs_bbx(region.min, region.max, 0),
                end = octree_.end_leafs_bbx(); it != end; ++it) {
                callback(&it);
            }
        }
    }
}

bool OctoMapServer::isInMoveDifferenceRegion(
        const Point& original_min,
        const Point& original_max,
        const Point& moved_distance,
        const Point& point) {

    // 移动后的包围盒
    octomap::point3d moved_min = original_min + moved_distance;
    octomap::point3d moved_max = original_max + moved_distance;

    // 检查点是否在原始包围盒内但不在移动后包围盒内（即removed区域）
    bool in_original = (point.x() >= original_min.x() && point.x() <= original_max.x()) &&
                      (point.y() >= original_min.y() && point.y() <= original_max.y()) &&
                      (point.z() >= original_min.z() && point.z() <= original_max.z());

    bool in_moved = (point.x() >= moved_min.x() && point.x() <= moved_max.x()) &&
                   (point.y() >= moved_min.y() && point.y() <= moved_max.y()) &&
                   (point.z() >= moved_min.z() && point.z() <= moved_max.z());

    return in_original && !in_moved;  // L形区域 = 原始区域 - 移动后区域
}

std::vector<OctoMapServer::BoundingRegion> OctoMapServer::calculateMoveDifferenceRegions(
    const Point& original_min,
    const Point& original_max,
    const Point& moved_distance) {

    std::vector<BoundingRegion> regions;

    // Calculate moved bounding box
    Point moved_min = original_min + moved_distance;
    Point moved_max = original_max + moved_distance;

    // Region 1: X direction difference (left/right strip)
    if (moved_distance.x() > 0) {
        // Movement to right, left strip is removed
        if (moved_min.x() > original_min.x()) {
            regions.push_back(BoundingRegion(
                Point(original_min.x(), original_min.y(), original_min.z()),
                Point(moved_min.x(), original_max.y(), original_max.z())
            ));
        }
    } else if (moved_distance.x() < 0) {
        // Movement to left, right strip is removed
        if (moved_max.x() < original_max.x()) {
            regions.push_back(BoundingRegion(
                Point(moved_max.x(), original_min.y(), original_min.z()),
                Point(original_max.x(), original_max.y(), original_max.z())
            ));
        }
    }

    // Calculate the remaining area for Y and Z direction checks
    double remaining_min_x = std::max(original_min.x(), moved_min.x());
    double remaining_max_x = std::min(original_max.x(), moved_max.x());

    // Region 2: Y direction difference (front/back strip)
    if (remaining_min_x <= remaining_max_x) {
        if (moved_distance.y() > 0) {
            // Movement to back, front strip is removed
            if (moved_min.y() > original_min.y()) {
                regions.push_back(BoundingRegion(
                    Point(remaining_min_x, original_min.y(), original_min.z()),
                    Point(remaining_max_x, moved_min.y(), original_max.z())
                ));
            }
        } else if (moved_distance.y() < 0) {
            // Movement to front, back strip is removed
            if (moved_max.y() < original_max.y()) {
                regions.push_back(BoundingRegion(
                    Point(remaining_min_x, moved_max.y(), original_min.z()),
                    Point(remaining_max_x, original_max.y(), original_max.z())
                ));
            }
        }
    }

    // Calculate the remaining area for Z direction check
    double remaining_min_y = std::max(original_min.y(), moved_min.y());
    double remaining_max_y = std::min(original_max.y(), moved_max.y());

    // Region 3: Z direction difference (bottom/top strip)
    if (remaining_min_x <= remaining_max_x && remaining_min_y <= remaining_max_y) {
        if (moved_distance.z() > 0) {
            // Movement up, bottom strip is removed
            if (moved_min.z() > original_min.z()) {
                regions.push_back(BoundingRegion(
                    Point(remaining_min_x, remaining_min_y, original_min.z()),
                    Point(remaining_max_x, remaining_max_y, moved_min.z())
                ));
            }
        } else if (moved_distance.z() < 0) {
            // Movement down, top strip is removed
            if (moved_max.z() < original_max.z()) {
                regions.push_back(BoundingRegion(
                    Point(remaining_min_x, remaining_min_y, moved_max.z()),
                    Point(remaining_max_x, remaining_max_y, original_max.z())
                ));
            }
        }
    }

    return regions;
}

} // namespace finenav_2d

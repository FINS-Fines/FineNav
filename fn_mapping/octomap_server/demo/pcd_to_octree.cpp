// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "octomap_server.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace finenav_2d;

int main(int argc, char** argv) {
    constexpr bool use_height_octree = true;

    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::io::loadPCDFile("/home/huigg/Desktop/nav_ws/final_map.pcd", point_cloud);
    std::cout << "Loaded " << point_cloud.size() << " points from PCD file." << std::endl;

    if (use_height_octree) {
        OctoMapServer octomap_server(0.02);
        for (const auto& point : point_cloud.points) {
            octomap_server.getOctree().updateNodeWithHeight(octomap::point3d(point.x, point.y, point.z), point.z);
        }
        octomap_server.getOctree().write("/home/huigg/Desktop/nav_ws/final_map.ot");
        std::cout << "Saved " << octomap_server.getOctree().size() << " nodes to octomap file." << std::endl;

    } else {
        octomap::OcTree octree(0.02);
        for (const auto& point : point_cloud.points) {
            octree.updateNode(octomap::point3d(point.x, point.y, point.z), true); // 设置为占据
        }
        octree.updateInnerOccupancy(); // 更新内部节点的占据概率
        octree.write("/home/huigg/Desktop/nav_ws/final_map.ot");
        std::cout << "Saved " << octree.size() << " nodes to octomap file." << std::endl;

    }

    return 0;
}

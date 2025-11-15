// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "octomap_server.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace finenav_2d;

int main(int argc, char** argv) {

    OctoMapServer octomap_server(0.02); // 分辨率0.05米
    auto result = octomap_server.openFile("/home/fins/Desktop/Nav_ws/final_map.ot");
    if (!result) {
        std::cerr << "Can't open file" << std::endl;
        return -1;
    }
    std::cout << "Opened with size " << octomap_server.getOctree().size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    const auto& tree = octomap_server.getOctree();
    for(octomap::HeightOcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it)
    {
        if (it->isHeightSet() && it.getDepth() == tree.getTreeDepth()) {
            point_cloud.push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it->getHeight()));
        }
    }
    pcl::io::savePCDFileBinary("/home/fins/Desktop/Nav_ws/final_map.pcd", point_cloud);
    std::cout << "Saved " << point_cloud.size() << " points to PCD file." << std::endl;




    return 0;
}

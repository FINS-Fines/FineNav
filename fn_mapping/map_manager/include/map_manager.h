// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_MAP_MANAGER_H
#define FINENAV2D_MAP_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "grid_map.hpp"

namespace finenav_2d {

class MapManager : public rclcpp::Node {
public:
    explicit MapManager(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief 管理主流程
     */
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief 发布局部地图
     */
    void publishLocalMap();



    // MapManager需要管理全局地图和代价地图，这里通过依赖注入的方式实现
    GridMap<bool> local_map_; // TODO: 后续改为依赖注入

    // Input1: 监听tf，map，base_link

    // Input2: pcd_cbk

};

}

#endif //FINENAV2D_MAP_MANAGER_H

// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_MAP_MANAGER_H
#define FINENAV2D_MAP_MANAGER_H

#include <rclcpp/rclcpp.hpp>

#include "grid_map.hpp"

namespace finenav_2d {

class MapManager : public rclcpp::Node {
public:
    explicit MapManager(const rclcpp::NodeOptions& options);

    /**
     * @brief 发布局部地图
     */
    void publishLocalMap();

    /**
     * @brief 发布全局地图
     */
    void publishGlobalMap();

private:
    // MapManager需要管理全局地图和代价地图，这里通过依赖注入的方式实现
    std::shared_ptr<GridMap<bool>> global_map_;

};

}

#endif //FINENAV2D_MAP_MANAGER_H

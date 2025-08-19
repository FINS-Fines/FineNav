// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_PCT_PLANNER_HPP
#define FINENAV2D_PCT_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include "Tomography.hpp"

using namespace finenav_2d;

class PctPlanner : public rclcpp::Node {
public:
    explicit PctPlanner(const rclcpp::NodeOptions& options);

    //TODO: 回调，接收目标位姿，启动A*规划，发布规划后的path

private:
    std::shared_ptr<Tomography> tomography_;

};

#endif //FINENAV2D_PCT_PLANNER_HPP

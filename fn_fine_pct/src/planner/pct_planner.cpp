// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "pct_planner.hpp"

PctPlanner::PctPlanner(const rclcpp::NodeOptions& options) : Node(options.arguments()[0], options) {
    // 从ROS2参数服务器读取参数

    // 动态构造tomography

    // 初始化Astar
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.arguments({"fn_global_map_node"});
    auto node = std::make_shared<PctPlanner>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

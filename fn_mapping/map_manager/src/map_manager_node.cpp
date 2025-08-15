// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "map_manager.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.arguments({"map_manager_node"});
    auto node = std::make_shared<finenav_2d::MapManager>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

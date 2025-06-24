// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "rclcpp/rclcpp.hpp"
#include "localization_manager.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.arguments({"localization_manager_node"});
    auto node_ptr = std::make_shared<finenav_2d::LocalizationManager>(options);
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}

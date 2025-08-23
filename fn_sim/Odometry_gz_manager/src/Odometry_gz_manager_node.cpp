// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 25-8-22.
//
#include "rclcpp/rclcpp.hpp"
#include "Odometry_gz_manager.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.arguments({"Odometry_gz__manager_node"});
    auto node_ptr = std::make_shared<finenav_2d::Odometry_gz_manager>(options);
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
}

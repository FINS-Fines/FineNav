#pragma once

#include <string>

struct Config {
    std::string input_topic = "/pointcloud";
    std::string output_path = "./output";
    // 添加其他配置参数
};
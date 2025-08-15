#pragma once
#include "Tomography.hpp"
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <utility>

class Planner {
public:
    explicit Planner(std::shared_ptr<Tomography> tomography);
    nav_msgs::msg::Path planPath(const std::array<float, 2>& start, const std::array<float, 2>& end);

private:
    std::shared_ptr<Tomography> tomography_;

    // 使用更清晰的数据结构存储路径信息
    struct path_node {
        int x, y, layer;
        float g, h;
        bool operator<(const path_node& other) const {
            return (g + h) > (other.g + other.h);
        }
    };

    // 修改 came_from 类型为包含层信息
    using CameFromMap = std::unordered_map<
        int,  // layer
        std::unordered_map<
            int,  // x
            std::unordered_map<
                int,  // y
                path_node  // parent node
            >
        >
    >;
    CameFromMap came_from_;

    // Helper methods
    std::pair<int, int> worldToGrid(float x, float y) const;
    std::pair<float, float> gridToWorld(int i, int j) const;
    float heuristic(int x1, int y1, int x2, int y2) const;
    bool isTraversable(int x, int y, int layer) const;

    std::pair<int, int> findBestStartEndLayers(int start_x, int start_y, int end_x, int end_y) const;
};
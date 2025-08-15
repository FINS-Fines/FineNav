#pragma once
#include "Tomography.hpp"
#include <nav_msgs/msg/path.hpp>
#include "PlannerConfig.hpp"
#include <vector>
#include <utility>

struct NodeCost {
    float traversal_cost;
    float height_change_cost;
    float total_cost;
};

class Planner {
public:
    explicit Planner(std::shared_ptr<Tomography> tomography,const PlannerConfig::Params& cfg);
    nav_msgs::msg::Path planPath(const std::array<float, 2>& start, const std::array<float, 2>& end);

private:
    std::shared_ptr<Tomography> tomography_;

    // 使用更清晰的数据结构存储路径信息
    struct path_node {
        int x, y, layer;
        float height;
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

    PlannerConfig::Params cfg_;

    // Helper methods
    std::pair<int, int> worldToGrid(float x, float y) const;
    std::pair<float, float> gridToWorld(int i, int j) const;
    float heuristic(int x1, int y1, int z1, int x2, int y2, int z2) const ;
    bool isTraversable(int x, int y, int layer) const;
    bool isValidHeightChange(float from_height, float to_height, float horizontal_dist) const;
    bool isValidTransition(const path_node& from, int to_x, int to_y, int to_layer) const;

    std::pair<int, int> findBestStartEndLayers(int start_x, int start_y, int end_x, int end_y) const;
};
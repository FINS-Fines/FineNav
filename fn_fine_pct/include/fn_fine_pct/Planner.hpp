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

    // Helper methods
    std::pair<int, int> worldToGrid(float x, float y) const;
    std::pair<float, float> gridToWorld(int i, int j) const;
    float heuristic(int x1, int y1, int x2, int y2) const;
    bool isTraversable(int x, int y, int layer) const;
};
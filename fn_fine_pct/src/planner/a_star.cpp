// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#include <iostream>
#include <chrono>
#include "a_star.hpp"

namespace finenav_2d {


Astar::Astar(){

    nearby_cells_offset_ = { // 8 neighbors in 2D
        Index(0, -1, -1), Index(0, -1, 0), Index(0, -1, 1),
        Index(0, 0, -1),  Index(0, 0, 1), Index(0, 1, -1),
        Index(0, 1, 0),   Index(0, 1, 1),
    };

    nearby_layers_offset_ = { // 3 layers in 3D
        Index(0, 0, 0), Index(1, 0, 0), Index(-1, 0, 0),
    }; // TODO: 需要包括自己吗
}

/**
 * @brief 初始化A*的地图
 */
void Astar::initialize(const double cost_threshold, const double step_cost_weight, const TomographyLayers& tomography,  HeuristicType h_type) {
    tomography_ = tomography;
    max_layers_ = tomography.trav_cost.layers.size();
    max_x_ = tomography.trav_cost.layers[0].rows();
    max_y_ = tomography.trav_cost.layers[0].cols();
    h_type_ = h_type;

    // TODO: 搜索过程中单独判断NAN
}

bool Astar::search(const Index& start, const Index& goal) {
    path_.clear();




}


// TODO: 检查这里的启发函数实现是否正确
double Astar::getHeuristic(const Node* node1, const Node* node2) const {
    double cost = 0.0;

    if (h_type_ == kEuclidean) {
        // l2 distance
        cost = (node1->idx - node2->idx).norm();
    } else if (h_type_ == kDiagonal) {
        // octile distance
        Eigen::Vector3i d = node1->idx - node2->idx;
        int dx = abs(d(0)), dy = abs(d(1)), dz = abs(d(2));
        int dmin = std::min(dx, std::min(dy, dz));
        int dmax = std::max(dx, std::max(dy, dz));
        int dmid = dx + dy + dz - dmin - dmax;
        double h =
            std::sqrt(3) * dmin + std::sqrt(2) * (dmid - dmin) + (dmax - dmid);
        cost = h;
    } else if (h_type_ == kManhattan) {
        cost = (node1->idx - node2->idx).lpNorm<1>();
    } else {
        assert(false && "not implemented");
    }

    // cost += std::abs(node1->idx[0] - node2->idx[0]) * 10;
    return cost;
}




}
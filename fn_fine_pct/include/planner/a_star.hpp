// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_A_STAR_HPP
#define FINENAV2D_A_STAR_HPP

#include <limits>
#include <Eigen/Core>
#include "Tomography.hpp"

namespace finenav_2d {

// TODO: 专门放到一个tyde_defs中

using Index = Eigen::Vector3i;
using Path = std::vector<Index>;

enum HeuristicType : int {
    kEuclidean,
    kManhattan,
    kDiagonal,
};


class Node {
public:
    Node() = default;
    Node(Index idx, Node* parent) : idx(idx), parent(parent) {}
    ~Node() = default;

    bool operator==(const Node& other) const { return idx == other.idx; }

    void reset() {
        g = std::numeric_limits<double>::max();
        f = std::numeric_limits<double>::max();
        parent = nullptr;
    }

    // TODO: tomography的idx是layer, col, row，必须保持一致；顺便给MapLayers做一个Index索引的接口

    Index idx = Index(0, 0, 0);  // layer, row, col
    Node* parent = nullptr;
    double g = std::numeric_limits<double>::max(); // 累积到该节点的g(n)
    double f = std::numeric_limits<double>::max(); // 估计的总代价f(n) = g(n) + h(n)

    // 真的需要吗？

    double height = 0.0; // 高度
    double cost = 0.0; // 代价
    int layer = 0;
};

struct NodeCompare {
  bool operator()(const Node* a, const Node* b) const { return a->f > b->f; }
};

using MultiLayerGridMap = std::vector<std::vector<std::vector<Node>>>;

class Astar {
public:
    explicit Astar();
    ~Astar() = default;

    // TODO: 包装一个config，一口气给Astar参数，所有参数都应该可以在算法初始化时重置
    void initialize(const double cost_threshold, const double step_cost_weight, const TomographyLayers& tomography, const HeuristicType h_type);

    /**
     * @brief A*搜索，搜索后可由getPath()获取搜索路径
     * @param start MapLayers起始点索引
     * @param goal MapLayers终点索引
     * @return 如果找到路径，返回true
     */
    bool search(const Index& start, const Index& goal);

    Path getPath() const { return path_; }

private:
    double getHeuristic(const Node* node1, const Node* node2) const;

    int getHash(const Index idx) const;
    int decideLayer(const Node* cur_node) const;
    void reset();



    int max_x_ = 0;
    int max_y_ = 0;
    int max_layers_ = 0;
    double cost_threshold_ = 50;
    double step_cost_weight_ = 1.0;

    MultiLayerGridMap grid_map_;

    int search_layer_depth_ = 1;
    std::vector<int> search_layers_offset_;
    std::vector<Index> nearby_cells_offset_;
    std::vector<Index> nearby_layers_offset_;
    


    HeuristicType h_type_ = kEuclidean;

    TomographyLayers tomography_;
    Path path_;


};

}



#endif //FINENAV2D_A_STAR_HPP

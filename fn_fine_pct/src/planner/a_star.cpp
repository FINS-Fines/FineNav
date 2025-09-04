// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#include "a_star.hpp"

#include <iostream>
#include <chrono>
#include <queue>
#include <unordered_map>
#include <unordered_set>


namespace finenav_2d {


Astar::Astar(){

    nearby_cells_offset_ = { // 8 neighbors in 2D
        Index(0, -1, -1), Index(0, -1, 0), Index(0, -1, 1),
        Index(0, 0, -1),  Index(0, 0, 1), Index(0, 1, -1),
        Index(0, 1, 0),   Index(0, 1, 1), 
        Index(1, 0, 0),   Index(1, -1, 0), Index(1, 1, 0),
        Index(1, 0, -1),  Index(1, 0, 1), Index(1, -1, -1),
        Index(1, -1, 1),  Index(1, 1, -1), Index(1, 1, 1),

        Index(-1, 0, 0),  Index(-1, -1, 0), Index(-1, 1, 0),
        Index(-1, 0, -1), Index(-1, 0, 1), Index(-1, -1, -1),
        Index(-1, -1, 1), Index(-1, 1, -1), Index(-1, 1, 1)
    };

    nearby_layers_offset_ = { // 3 layers in 3D
        Index(0, 0, 0), Index(1, 0, 0), Index(-1, 0, 0),
    }; // TODO: 需要包括自己吗
}

/**
 * @brief 初始化A*的地图
 */
void Astar::initialize(const double cost_threshold, const double step_cost_weight, const TomographyLayers& tomography,  HeuristicType h_type) {
    printf("Astar initializing...\n");
    tomography_ = tomography;
    max_layers_ = tomography.trav_cost.layers.size();
    max_x_ = tomography.trav_cost.layers[0].cols();
    max_y_ = tomography.trav_cost.layers[0].rows();

    h_type_ = h_type;
    // printf("Astar max_layers_: %d, max_x_: %d, max_y_: %d\n", max_layers_, max_x_, max_y_);
    grid_map_.resize(max_layers_);
    for (size_t i = 0; i < max_layers_; ++i) {
        grid_map_[i].resize(max_y_);
        for (size_t j = 0; j < max_y_; ++j) { // y是rows
            grid_map_[i][j].resize(max_x_);
            for (size_t k = 0; k < max_x_; ++k) { // x是cols
                // 从 TomographyLayer 获取数据
                double cost = tomography.trav_cost.layers[i](j, k);
                double ground_height = tomography.ground.layers[i](j, k);
                double ceiling_height = tomography.ceiling.layers[i](j, k);

                // 处理ground和ceiling中的 NaN 值
                if (std::isnan(ground_height)) { ground_height = std::numeric_limits<float>::lowest(); }
                if (std::isnan(ceiling_height)) { ceiling_height =  std::numeric_limits<float>::max(); }

                // 计算离散化的高度索引
                // double z = static_cast<int>(ground_height / resolution_); // 这里是因为xy是已经被index化了，这里需要把z也给同样index化
                // Astar max_layers_: 8, max_x_: 150, max_y_: 285
                // 创建节点
                // grid_map_[i][j][k] = Node(Eigen::Vector3i(z, j, k), nullptr); // 这里的i是layer，j是row，k是col，z是高度
                grid_map_[i][j][k] = Node(Eigen::Vector3i(i, j, k), nullptr); // 这里的i是layer，j是row，k是col，z是高度
                grid_map_[i][j][k].cost = cost;
                grid_map_[i][j][k].height = ground_height;
                grid_map_[i][j][k].layer = i;
                // if (ground_height > 0 && cost < 20 && i == 1)
                // {
                //   printf("Node at layer %d, row %d, col %d initialized with cost %.2f and height %.2f\n", i, j, k, cost, ground_height);
                // }
                
                
                // 计算 ele (高程差信息)
                // 使用天花板和地面的高度差作为 ele
                // double interval = ceiling_height - ground_height;
                // if (std::isfinite(interval) && interval > 0) {
                //     grid_map_[i][j][k].ele = interval;
                // } else {
                //     grid_map_[i][j][k].ele = 0.0;
                // }
            }
        }
    }

    search_layers_offset_.clear(); 
    // search_layers_offset_.emplace_back(0); // 包括当前层
    for (int i = 0; i < search_layer_depth_; ++i) {
        search_layers_offset_.emplace_back(-(i + 1));
        search_layers_offset_.emplace_back(i + 1);
    }


    // TODO: 搜索过程中单独判断NAN
}

void Astar::reset() {
    for (size_t i = 0; i < grid_map_.size(); ++i) {
        for (size_t j = 0; j < grid_map_[i].size(); ++j) {
            for (size_t k = 0; k < grid_map_[i][j].size(); ++k) {
                grid_map_[i][j][k].reset();
            }
        }
    }
}

int Astar::getHash(const Index idx) const {
    return idx[0] * 10000000 + idx[1] * max_x_ + idx[2];
}

bool Astar::search(const Index& start, const Index& goal) {
  printf("Astar searching...\n");
    path_.clear(); // 清空上次搜索结果

    auto start_node = &grid_map_[start[0]][start[1]][start[2]]; // TODO: 这里的索引顺序可能有问题，需要确认
    auto goal_node = &grid_map_[goal[0]][goal[1]][goal[2]];     // TODO: 增加一个逻辑可以将start和goal的z标定到合理的高度上去, 这里将点直接固定到每个层中的一个小节点中
    // 需要把目标转换到我的grid——map中的一个指定node上去，这里的resolution是相当于你输入的z坐标落到的层上，就是每一层的高度
    printf("Astar start node: layer %d, row %d, col %d, cost %.2f, height %.2f\n", start_node->idx[0], start_node->idx[1], start_node->idx[2], start_node->cost, start_node->height);
    printf("Astar goal node: layer %d, row %d, col %d, cost %.2f, height %.2f\n", goal_node->idx[0], goal_node->idx[1], goal_node->idx[2], goal_node->cost, goal_node->height);
    start_node->g = 0.0;

    if (start_node->cost > cost_threshold_ || goal_node->cost > cost_threshold_) {
        printf("start or goal node cost too high, cannot find path\n");
        return false;
    }

    std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
    std::unordered_map<int, Node*> closed_set; // 已访问节点

    open_set.push(start_node);

    printf("start searching\n");

    // A star 主循环
  while (!open_set.empty()) {
    // 1. 取出最优节点
    Node* current_node = open_set.top();
    printf("current node: layer %d, row %d, col %d, g %.2f, f %.2f, cost %.2f, height %.2f\n",
           current_node->idx[0], current_node->idx[1], current_node->idx[2],
           current_node->g, current_node->f, current_node->cost, current_node->height);
    open_set.pop();

    // 2. 检查是否到达目标，若到达了则从fringe中回溯路径
    if (current_node->idx == goal_node->idx) {
      while (current_node->parent != nullptr) {
        // search_result_.emplace_back(Eigen::Vector3i(
        //     current_node->layer, current_node->idx[1],
        //     current_node->idx[2]));
        // 将idx中的layer值修改为height值
        current_node->idx[0] = static_cast<int>(current_node->height * 100);
        path_.emplace_back(current_node->idx);
        current_node = current_node->parent;
      }
      // 从终点开始回溯，逆转下从起点开始排序
      std::reverse(path_.begin(), path_.end());
      printf("path found, path length: %lu\n", path_.size());
      return true;
    }

    // 3. 记录该节点已被访问
    closed_set[getHash(current_node->idx)] = current_node;

    // 4. 决定Layer
    // int layer = current_node->layer;
    // if (current_node->ele > 0.5) {
    //   layer = std::min(layer + 1, max_layers_ - 1);
    // } else if (current_node->ele < -0.5) {
    //   layer = std::max(layer - 1, 0);
    // }


    // 5. 拓展邻居节点
    int i, j, layer = 0;
    double height_current = current_node->height;
    double tentative_g = 0.0;
    for (const auto& neighbor : nearby_cells_offset_) {
      
      i = current_node->idx[1] + neighbor[1]; // x 方向偏移
      j = current_node->idx[2] + neighbor[2]; // y 方向偏移
      layer = current_node->idx[0] + neighbor[0]; // layer 方向偏移


      if (i < 0 || i >= max_y_ || j < 0 || j >= max_x_ || layer < 0 || layer >= max_layers_) {
        continue;
      }

      auto neighbor_node = &grid_map_[layer][i][j];

      // TODO: 完全不明所以的配置逻辑
    //   if (neighbor_node->cost > cost_threshold_) { // 邻居节点的代价是否超过阈值
    //     if (abs(neighbor_node->ele) < 0.5) { // 如果代价超过了，��且高度差0.5？？？？？
    //       continue;
    //     } else {
    //       if (std::abs(neighbor_node->height - current_node->height) > 0.3) {
    //         continue;
    //       }
    //     }
    //   }

      if ((neighbor_node->cost > cost_threshold_) ||
          std::abs(neighbor_node->height - current_node->height) > 1.5) {
        continue;
      }

      // TODO: 这个step_cost在这也是奇怪，tentative_g是啥也不清楚
      auto diff = neighbor_node->idx - current_node->idx;
      double step_cost = step_cost_weight_ * neighbor_node->cost;
      if (step_cost < 5) step_cost = 0.0;
      tentative_g =
          current_node->g +
          std::sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]) +
          step_cost;

      // 这里的逻辑大概是这样的：比如说走对角线时候，你需要考虑到斜边的距离，以及地块的cost。因此你要考虑的就会更多

      // 如果邻居节点已经在closed_set中，且g值更大，则跳过
      auto p_neighbor = closed_set.find(getHash(neighbor_node->idx));
      if (p_neighbor != closed_set.end()) {
        if (tentative_g >= p_neighbor->second->g) {
          continue;
        }
      }

      // 更新邻居节点
      if (tentative_g < neighbor_node->g) {
        neighbor_node->g = tentative_g;
        neighbor_node->f = tentative_g + getHeuristic(neighbor_node, goal_node);
        neighbor_node->parent = current_node;
        open_set.push(neighbor_node);
      }
    }
  }
  printf("no path found\n");
  return false;

}

// int Astar::decideLayer(const Node* cur_node) const {
//   int layer = cur_node->layer;
//   int i = cur_node->idx[1];
//   int j = cur_node->idx[2];
//   double cur_height = cur_node->height;

//   int true_layer = layer;

//   for (const auto offset : search_layers_offset_) {
//     printf("  trying layer offset: %d\n", offset);
//     int cur_layer = layer + offset;

//     if (cur_layer < 0 || cur_layer >= max_layers_) {
//       continue;
//     }

//     const Node& search_node = grid_map_[cur_layer][i][j];

//     if (abs(search_node.height - cur_height) > 0.2) {
//       continue;
//     }

//     true_layer = cur_layer;
//     break;

//     // 直接变换层数，应该就可以

//     // if (search_node.ele > 0.5) {
//     //   true_layer = std::min(cur_layer + 1, max_layers_ - 1);
//     //   break;
//     // } else if (search_node.ele < -0.5) {
//     //   true_layer = std::max(cur_layer - 1, 0);
//     //   break;
//     // }
//   }

//   return true_layer;
// }
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
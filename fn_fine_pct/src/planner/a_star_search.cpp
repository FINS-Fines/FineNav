// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "a_star_search.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "type_defs.hpp"

using std::cout;
using std::endl;

// 9 neighbors in 2d
static std::vector<Eigen::Vector2i> kNeighbors = std::vector<Eigen::Vector2i>{
    Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, 1),
    Eigen::Vector2i(0, -1),  Eigen::Vector2i(0, 1),  Eigen::Vector2i(1, -1),
    Eigen::Vector2i(1, 0),   Eigen::Vector2i(1, 1),
};

/**
 * @brief 初始化A*的地图
 */
void Astar::Init(const double cost_threshold, const double resolution, const double step_cost_weight, const std::vector<TomographyLayer>& layers) {
    auto t0 = std::chrono::high_resolution_clock::now(); // 计时

    cost_threshold_ = cost_threshold; // 可通行的代价阈值
    step_cost_weight_ = step_cost_weight;  // 跨层代价权重
    resolution_ = resolution;  

    max_layers_ = layers.size();
    max_x_ = layers[0].trav_cost.cols();
    max_y_ = layers[0].trav_cost.rows();
    // xy_size_ = max_x_ * max_y_;

    // 初始化 grid_map_
    grid_map_.resize(max_layers_);
    for (size_t i = 0; i < max_layers_; ++i) {
        grid_map_[i].resize(max_y_);
        for (size_t j = 0; j < max_y_; ++j) {
            grid_map_[i][j].resize(max_x_);
            for (size_t k = 0; k < max_x_; ++k) {
                // 从 TomographyLayer 获取数据
                double cost = layers[i].trav_cost(j, k);
                double ground_height = layers[i].ground(j, k);
                double ceiling_height = layers[i].ceiling(j, k);

                // 处理ground和ceiling中的 NaN 值
                if (std::isnan(ground_height)) { ground_height = std::numeric_limits<float>::lowest(); }
                if (std::isnan(ceiling_height)) { ceiling_height =  std::numeric_limits<float>::max(); }

                // 计算离散化的高度索引
                double z = static_cast<int>(ground_height / resolution);

                // 创建节点
                grid_map_[i][j][k] = Node(Eigen::Vector3i(z, j, k), nullptr);
                grid_map_[i][j][k].cost = cost;
                grid_map_[i][j][k].height = ground_height;
                grid_map_[i][j][k].layer = i;

                // 计算 ele (高程差信息)
                // 使用天花板和地面的高度差作为 ele
                double interval = ceiling_height - ground_height;
                if (std::isfinite(interval) && interval > 0) {
                    grid_map_[i][j][k].ele = interval;
                } else {
                    grid_map_[i][j][k].ele = 0.0;
                }
            }
        }
    }


    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - t0);

    search_layers_offset_.clear(); 
    search_layers_offset_.emplace_back(0); // 包括当前层
    for (int i = 0; i < search_layer_depth_; ++i) {
        search_layers_offset_.emplace_back(-(i + 1));
        search_layers_offset_.emplace_back(i + 1);
    }

    printf("Astar initialized with TomographyLayers, max_x: %d, max_y: %d, max_layers: %d, time elapsed: %f ms\n",
           max_x_, max_y_, max_layers_, duration.count() / 1000.0);
}

void Astar::Reset() {
  for (size_t i = 0; i < grid_map_.size(); ++i) {
    for (size_t j = 0; j < grid_map_[i].size(); ++j) {
      for (size_t k = 0; k < grid_map_[i][j].size(); ++k) {
        grid_map_[i][j][k].Reset();
      }
    }
  }
}

int Astar::GetHash(const Eigen::Vector3i& idx) const {
  return idx[0] * 10000000 + idx[1] * max_x_ + idx[2];
}

bool Astar::Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) {
  auto t0 = std::chrono::high_resolution_clock::now();// 计时

  if (!search_result_.empty()) {
    Reset();
    search_result_.clear();
  } // 清空上次搜索结果

  auto start_node = &grid_map_[start[0]][start[2]][start[1]];
  auto goal_node = &grid_map_[goal[0]][goal[2]][goal[1]];
  start_node->g = 0.0;

  if (goal_node->cost > cost_threshold_) {
    printf("goal node is not reachable, cost: %f", goal_node->cost);
    return false;
  }

  std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set; // 搜索树待扩展的节点指针，优先队列存储
  std::unordered_map<int, Node*> closed_set; // 存储已探索完成的节点指针，用哈希表实现

  open_set.push(start_node);

  printf("start searching\n");

  // A star 主循环
  while (!open_set.empty()) {
    // 1. 取出最优节点
    Node* current_node = open_set.top();
    open_set.pop();

    // 2. 检查是否到达目标，若到达了则从fringe中回溯路径
    if (current_node->idx == goal_node->idx) {
      while (current_node->parent != nullptr) {
        // search_result_.emplace_back(Eigen::Vector3i(
        //     current_node->layer, current_node->idx[1],
        //     current_node->idx[2]));
        search_result_.emplace_back(current_node);
        current_node = current_node->parent;
      }
      // 从终点开始回溯，逆转下从起点开始排序
      std::reverse(search_result_.begin(), search_result_.end());
      if (debug_) ConvertClosedSetToMatrix(closed_set);
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - t0);
      printf("path found, time elapsed: %f ms\n",
             duration.count() / 1000.0);
      return true;
    }

    // 3. 记录该节点已被访问
    closed_set[GetHash(current_node->idx)] = current_node;

    // 4. 决定Layer
    // int layer = current_node->layer;
    // if (current_node->ele > 0.5) {
    //   layer = std::min(layer + 1, max_layers_ - 1);
    // } else if (current_node->ele < -0.5) {
    //   layer = std::max(layer - 1, 0);
    // }
    int layer = DecideLayer(current_node); // 要不要跳到下一个layer

    // 5. 拓展邻居节点
    int i, j = 0;
    double tentative_g = 0.0;
    for (const auto& neighbor : kNeighbors) {
      i = current_node->idx[1] + neighbor[0]; // x 方向偏移
      j = current_node->idx[2] + neighbor[1]; // y 方向偏移

      if (i < 0 || i >= max_y_ || j < 0 || j >= max_x_) {
        continue;
      }

      auto neighbor_node = &grid_map_[layer][i][j];

      // TODO: 完全不明所以的配置逻辑
      if (neighbor_node->cost > cost_threshold_) { // 邻居节点的代价是否超过阈值
        if (abs(neighbor_node->ele) < 0.5) { // 如果代价超过了，��且高度差0.5？？？？？
          continue;
        } else {
          if (std::abs(neighbor_node->height - current_node->height) > 0.3) {
            continue;
          }
        }
      }

      // if ((neighbor_node->cost > cost_threshold_) ||
      //     std::abs(neighbor_node->height - current_node->height) > 0.3) {
      //   continue;
      // }

      // TODO: 这个step_cost在这也是奇怪，tentative_g是啥也不清楚
      auto diff = neighbor_node->idx - current_node->idx;
      double step_cost = step_cost_weight_ * neighbor_node->cost;
      if (step_cost < 5) step_cost = 0.0;
      tentative_g =
          current_node->g +
          std::sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]) +
          step_cost;

      // 如果邻居节点已经在closed_set中，且g值更大，则跳过
      auto p_neighbor = closed_set.find(GetHash(neighbor_node->idx));
      if (p_neighbor != closed_set.end()) {
        if (tentative_g >= p_neighbor->second->g) {
          continue;
        }
      }

      // 更新邻居节点
      if (tentative_g < neighbor_node->g) {
        neighbor_node->g = tentative_g;
        neighbor_node->f = tentative_g + GetHeuristic(neighbor_node, goal_node);
        neighbor_node->parent = current_node;
        open_set.push(neighbor_node);
      }
    }
  }

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);
  printf("path not found\n, time elapsed: %f ms\n",
         duration.count() / 1000.0);
  if (debug_) {
    ConvertClosedSetToMatrix(closed_set);
  }
  return false;
}

int Astar::DecideLayer(const Node* cur_node) const {
  int layer = cur_node->layer;
  int i = cur_node->idx[1];
  int j = cur_node->idx[2];
  double cur_height = cur_node->height;

  int true_layer = layer;

  for (const auto offset : search_layers_offset_) {
    int cur_layer = layer + offset;

    if (cur_layer < 0 || cur_layer >= max_layers_) {
      continue;
    }

    const Node& search_node = grid_map_[cur_layer][i][j];

    if (abs(search_node.height - cur_height) > 0.2) {
      continue;
    }

    if (search_node.ele > 0.5) {
      true_layer = std::min(cur_layer + 1, max_layers_ - 1);
      break;
    } else if (search_node.ele < -0.5) { // 哪来的 -0.5
      true_layer = std::max(cur_layer - 1, 0);
      break;
    }
  }

  return true_layer;
}

double Astar::CalculateStepCost(const Node* node1, const Node* node2) const {}

double Astar::GetHeuristic(const Node* node1, const Node* node2) const {
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

std::vector<PathPoint> Astar::GetPathPoints() const {
  std::vector<PathPoint> path_points;

  auto size = search_result_.size();
  path_points.resize(size);

  if (size == 0) {
    printf("path is empty\n, convert to path points failed\n");
    return path_points;
  }

  for (size_t i = 0; i < size; ++i) {
    // path_points[i].layer = search_result_[i][0];
    // path_points[i].x = search_result_[i][2];
    // path_points[i].y = search_result_[i][1];
    // if (i > 0) {
    //   path_points[i].heading =
    //       std::atan2(search_result_[i][1] - search_result_[i - 1][1],
    //                  search_result_[i][2] - search_result_[i - 1][2]);
    // }
    path_points[i].layer = search_result_[i]->layer;
    path_points[i].x = search_result_[i]->idx(2);
    path_points[i].y = search_result_[i]->idx(1);
    path_points[i].height = search_result_[i]->height;
    if (i > 0) {
      path_points[i].heading =
          std::atan2(search_result_[i]->idx(1) - search_result_[i - 1]->idx(1),
                     search_result_[i]->idx(2) - search_result_[i - 1]->idx(2));
    }
  }

  if (size > 1) {
    path_points[0].heading = path_points[1].heading;
  }

  return path_points;
}

Eigen::MatrixXd Astar::GetResultMatrix() const {
  if (search_result_.empty()) {
    printf("path is empty\n, convert to matrix failed\n");
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd path_matrix(search_result_.size(), 3);
  for (size_t i = 0; i < search_result_.size(); ++i) {
    path_matrix(i, 0) = search_result_[i]->layer;
    path_matrix(i, 1) = search_result_[i]->idx[1];
    path_matrix(i, 2) = search_result_[i]->idx[2];
  }
  return path_matrix;
}

void Astar::ConvertClosedSetToMatrix(
    const std::unordered_map<int, Node*>& closed_set) {
  visited_set_ = Eigen::MatrixXi(closed_set.size(), 3);
  int count = 0;
  for (auto i = closed_set.begin(); i != closed_set.end(); ++i) {
    visited_set_(count, 0) = i->second->layer;
    visited_set_(count, 1) = i->second->idx[1];
    visited_set_(count, 2) = i->second->idx[2];
    count += 1;
  }
}

std::vector<Eigen::Vector3i> Astar::GetNeighbors(Node* node) const {}

Eigen::MatrixXd Astar::GetCostLayer(int layer) const {
  Eigen::MatrixXd cost_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      cost_layer(i, j) = grid_map_[layer][i][j].cost;
    }
  }
  return cost_layer;
}
Eigen::MatrixXd Astar::GetEleLayer(int layer) const {
  Eigen::MatrixXd ele_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      ele_layer(i, j) = grid_map_[layer][i][j].ele;
    }
  }
  return ele_layer;
}

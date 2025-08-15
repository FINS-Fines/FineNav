#include "fn_fine_pct/Planner.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

Planner::Planner(std::shared_ptr<Tomography> tomography)
    : tomography_(tomography) {}

nav_msgs::msg::Path Planner::planPath(const std::array<float, 2>& start, const std::array<float, 2>& end) {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Clock().now();

    // 转换坐标
    auto [start_x, start_y] = worldToGrid(start[0], start[1]);
    auto [end_x, end_y] = worldToGrid(end[0], end[1]);

    // 查找最佳起始和目标层
    auto [start_layer, end_layer] = findBestStartEndLayers(start_x, start_y, end_x, end_y);

    RCLCPP_INFO(rclcpp::get_logger("planner"),
               "Planning from (%d,%d)@%d to (%d,%d)@%d",
               start_x, start_y, start_layer, end_x, end_y, end_layer);

    // A*算法实现
    std::priority_queue<path_node> open_set;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, bool>>> closed_set;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float>>> g_score;

    // 初始化
    open_set.push({start_x, start_y, start_layer, 0.0f, heuristic(start_x, start_y, end_x, end_y)});
    g_score[start_layer][start_x][start_y] = 0.0f;
    came_from_.clear();

    while (!open_set.empty()) {
        path_node current = open_set.top();
        open_set.pop();

        // 检查是否到达目标
        if (current.x == end_x && current.y == end_y && current.layer == end_layer) {
            // 重建路径
            std::vector<path_node> path_nodes;
            path_nodes.push_back(current);

            while (came_from_.count(current.layer) &&
                   came_from_[current.layer].count(current.x) &&
                   came_from_[current.layer][current.x].count(current.y)) {
                current = came_from_[current.layer][current.x][current.y];
                path_nodes.push_back(current);
            }

            // 转换为世界坐标
            for (auto it = path_nodes.rbegin(); it != path_nodes.rend(); ++it) {
                auto [wx, wy] = gridToWorld(it->x, it->y);
                float wz = tomography_->getSimplifiedHeightLayers()[it->layer][it->x][it->y];

                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.position.z = wz;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }

            RCLCPP_INFO(rclcpp::get_logger("planner"),
                       "Found path with %zu points", path.poses.size());
            return path;
        }

        closed_set[current.layer][current.x][current.y] = true;

        // 检查8邻域
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx = current.x + dx;
                int ny = current.y + dy;

                // 边界检查
                if (nx < 0 || nx >= tomography_->getMapDimX() ||
                    ny < 0 || ny >= tomography_->getMapDimY()) {
                    continue;
                }

                // 尝试当前层和相邻层
                for (int layer_offset = -1; layer_offset <= 1; ++layer_offset) {
                    int nl = current.layer + layer_offset;
                    if (nl < 0 || nl >= tomography_->getNumSimplifiedLayers()) {
                        continue;
                    }

                    // 检查可通行性
                    if (!isTraversable(nx, ny, nl)) {
                        continue;
                    }

                    // 计算代价
                    float movement_cost = sqrt(dx*dx + dy*dy);
                    float layer_change_cost = (layer_offset != 0) ? 1.0f : 0.0f;
                    float tentative_g = g_score[current.layer][current.x][current.y] +
                                      movement_cost + layer_change_cost;

                    // 如果节点已关闭或已有更低代价，跳过
                    if (closed_set[nl].count(nx) && closed_set[nl][nx].count(ny)) {
                        continue;
                    }

                    if (!g_score[nl].count(nx) || !g_score[nl][nx].count(ny) ||
                        tentative_g < g_score[nl][nx][ny]) {

                        // 记录路径
                        came_from_[nl][nx][ny] = current;
                        g_score[nl][nx][ny] = tentative_g;

                        float h = heuristic(nx, ny, end_x, end_y);
                        open_set.push({nx, ny, nl, tentative_g, h});

                        RCLCPP_DEBUG(rclcpp::get_logger("planner"),
                                   "Adding path_node (%d,%d)@%d, g=%.2f, h=%.2f",
                                   nx, ny, nl, tentative_g, h);
                    }
                }
            }
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("planner"), "No valid path found");
    return path;
}

std::pair<int, int> Planner::findBestStartEndLayers(int start_x, int start_y, int end_x, int end_y) const {
    int best_start_layer = 0;
    int best_end_layer = 0;
    float min_start_cost = std::numeric_limits<float>::max();
    float min_end_cost = std::numeric_limits<float>::max();

    // 遍历所有层，找到起点和终点最可通的层
    for (int l = 0; l < tomography_->getNumSimplifiedLayers(); ++l) {
        float start_cost = tomography_->getSimplifiedCostLayers()[l][start_x][start_y];
        float end_cost = tomography_->getSimplifiedCostLayers()[l][end_x][end_y];

        if (start_cost < min_start_cost && isTraversable(start_x, start_y, l)) {
            min_start_cost = start_cost;
            best_start_layer = l;
        }

        if (end_cost < min_end_cost && isTraversable(end_x, end_y, l)) {
            min_end_cost = end_cost;
            best_end_layer = l;
        }
    }

    return {best_start_layer, best_end_layer};
}

std::pair<int, int> Planner::worldToGrid(float x, float y) const {
    auto center = tomography_->getCenter();
    auto resolution = tomography_->getResolution();
    int map_dim_x = tomography_->getMapDimX();
    int map_dim_y = tomography_->getMapDimY();

    int i = static_cast<int>(std::round((x - center[0]) / resolution)) + map_dim_x / 2;
    int j = static_cast<int>(std::round((y - center[1]) / resolution)) + map_dim_y / 2;

    // 添加边界检查
    i = std::clamp(i, 0, map_dim_x - 1);
    j = std::clamp(j, 0, map_dim_y - 1);

    return {i, j};
}

std::pair<float, float> Planner::gridToWorld(int i, int j) const {
    auto center = tomography_->getCenter();
    auto resolution = tomography_->getResolution();
    int map_dim_x = tomography_->getMapDimX();
    int map_dim_y = tomography_->getMapDimY();

    float x = center[0] + (i - map_dim_x/2) * resolution;
    float y = center[1] + (j - map_dim_y/2) * resolution;

    return {x, y};
}

float Planner::heuristic(int x1, int y1, int x2, int y2) const {
    // Euclidean distance
    return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

bool Planner::isTraversable(int x, int y, int layer) const {
    // 检查层和坐标是否有效
    if (layer < 0 || layer >= tomography_->getNumSimplifiedLayers() ||
        x < 0 || x >= tomography_->getMapDimX() ||
        y < 0 || y >= tomography_->getMapDimY()) {
        return false;
        }

    // 获取该位置的代价
    float cost = tomography_->getSimplifiedCostLayers()[layer][x][y];

    // 定义可通行的代价阈值
    const float TRAVERSABLE_THRESHOLD = 50.0f; // 根据实际情况调整

    return cost < TRAVERSABLE_THRESHOLD;
}
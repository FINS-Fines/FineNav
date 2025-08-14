#include "fn_fine_pct/Planner.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>

Planner::Planner(std::shared_ptr<Tomography> tomography)
    : tomography_(tomography) {}

nav_msgs::msg::Path Planner::planPath(const std::array<float, 2>& start,
                                     const std::array<float, 2>& end) {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Clock().now();

    // Convert world coordinates to grid coordinates
    auto [start_x, start_y] = worldToGrid(start[0], start[1]);
    auto [end_x, end_y] = worldToGrid(end[0], end[1]);

    // A* algorithm implementation
    struct Node {
        int x, y;
        float g, h;
        bool operator<(const Node& other) const { return (g + h) > (other.g + other.h); }
    };

    std::priority_queue<Node> open_set;
    std::unordered_map<int, std::unordered_map<int, bool>> closed_set;
    std::unordered_map<int, std::unordered_map<int, std::pair<int, int>>> came_from;
    std::unordered_map<int, std::unordered_map<int, float>> g_score;

    // Initialize with start node
    open_set.push({start_x, start_y, 0.0f, heuristic(start_x, start_y, end_x, end_y)});
    g_score[start_x][start_y] = 0.0f;

    // For simplicity, we'll use the ground layer (layer 0)
    const int layer = 0;

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        if (current.x == end_x && current.y == end_y) {
            // Reconstruct path
            std::vector<std::pair<int, int>> path_points;
            path_points.emplace_back(end_x, end_y);

            while (came_from.count(path_points.back().first) &&
                   came_from[path_points.back().first].count(path_points.back().second)) {
                auto [x, y] = came_from[path_points.back().first][path_points.back().second];
                path_points.emplace_back(x, y);
            }

            // Convert to world coordinates and create Path message
            for (auto it = path_points.rbegin(); it != path_points.rend(); ++it) {
                auto [wx, wy] = gridToWorld(it->first, it->second);

                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.position.z = tomography_->getGroundHeight(layer, it->first, it->second);
                pose.pose.orientation.w = 1.0;

                path.poses.push_back(pose);
            }

            if (path_points.empty()) {
                RCLCPP_WARN(rclcpp::get_logger("planner"), "No path found!");
            }

            return path;
        }

        closed_set[current.x][current.y] = true;

        // Check all 8 neighbors
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx = current.x + dx;
                int ny = current.y + dy;

                // Check bounds
                if (nx < 0 || nx >= tomography_->getMapDimX() ||
                    ny < 0 || ny >= tomography_->getMapDimY()) {
                    continue;
                }

                // Check if already evaluated
                if (closed_set.count(nx) && closed_set[nx].count(ny)) {
                    continue;
                }

                // Check traversability
                if (!isTraversable(nx, ny, layer)) {
                    continue;
                }

                // Calculate tentative g score
                float tentative_g = g_score[current.x][current.y] +
                                   std::sqrt(dx*dx + dy*dy);

                if (!g_score.count(nx) || !g_score[nx].count(ny) ||
                    tentative_g < g_score[nx][ny]) {
                    came_from[nx][ny] = {current.x, current.y};
                    g_score[nx][ny] = tentative_g;
                    float h = heuristic(nx, ny, end_x, end_y);
                    open_set.push({nx, ny, tentative_g, h});
                }
            }
        }
    }

    // If we get here, no path was found
    return path;
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
    // 检查层数是否有效
    if (layer < 0 || layer >= tomography_->getNumLayers()) {
        RCLCPP_DEBUG(rclcpp::get_logger("planner"),
                    "Invalid layer index: %d", layer);
        return false;
    }

    // 检查坐标是否在网格范围内
    if (x < 0 || x >= tomography_->getMapDimX() ||
        y < 0 || y >= tomography_->getMapDimY()) {
        RCLCPP_DEBUG(rclcpp::get_logger("planner"),
                    "Coordinates out of bounds: (%d, %d)", x, y);
        return false;
    }

    // 获取代价并检查可通行性
    float cost = tomography_->getInflatedCost(layer, x, y);
    if (std::isnan(cost)) {
        RCLCPP_DEBUG(rclcpp::get_logger("planner"),
                    "NaN cost at (%d, %d, %d)", x, y, layer);
        return false;
    }

    return cost < tomography_->getConfig().cost_barrier * 0.5f;
}
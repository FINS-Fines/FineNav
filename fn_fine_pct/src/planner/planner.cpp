#include "fn_fine_pct/Planner.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include "fn_fine_pct/PlannerConfig.hpp"
#include <algorithm>

Planner::Planner(std::shared_ptr<Tomography> tomography,
                        const PlannerConfig::Params& cfg,
                        rclcpp::Logger logger,
                        rclcpp::Clock::SharedPtr clock)
: tomography_(tomography), cfg_(cfg) , logger_(logger),
      clock_(clock) {
    RCLCPP_INFO(logger_, "Planner initialized with clock type: %d",
                   clock_->get_clock_type());
}

nav_msgs::msg::Path Planner::planPath(const std::array<float, 2>& start, const std::array<float, 2>& end) {
    /********* 初始化 *********/
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = clock_->now();

    // 转换坐标
    auto [start_x, start_y] = worldToGrid(start[0], start[1]);
    auto [end_x, end_y] = worldToGrid(end[0], end[1]);

    // 查找最佳起始和目标层
    auto [start_layer, end_layer] = findBestStartEndLayers(start_x, start_y, end_x, end_y);

    RCLCPP_INFO(rclcpp::get_logger("planner"),
               "Planning from (%d,%d)@%d to (%d,%d)@%d",
               start_x, start_y, start_layer, end_x, end_y, end_layer);

    /********* A*算法实现 *********/
    std::priority_queue<path_node> open_set;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, bool>>> closed_set;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float>>> g_score;

    // 初始化 +++++++++++ NOTE Z
    open_set.push({start_x, start_y, start_layer, 0.0f, heuristic(start_x, start_y, 0 ,end_x, end_y,0)});
    g_score[start_layer][start_x][start_y] = 0.0f;
    came_from_.clear();

    while (!open_set.empty()) {
        path_node current = open_set.top();
        open_set.pop();

        // 情况1：找到目标 → 直接return，函数结束
        if (current.x == end_x && current.y == end_y && current.layer == end_layer) {
            std::vector<path_node> path_nodes;
            path_node node = current;
            while (true) {
                // 检查节点有效性
                float height = tomography_->getSimplifiedHeightLayers()[node.layer][node.x][node.y];
                if (std::isnan(height)) {
                    RCLCPP_ERROR(rclcpp::get_logger("planner"),
                                "NaN height at (%d,%d)@%d", node.x, node.y, node.layer);
                    break;
                }

                path_nodes.push_back(node);
                if (node.x == start_x && node.y == start_y && node.layer == start_layer) break;

                // 防止无限循环
                if (!came_from_[node.layer].count(node.x) || !came_from_[node.layer][node.x].count(node.y)) {
                    RCLCPP_WARN(rclcpp::get_logger("planner"),
                               "Path broken at (%d,%d)@%d", node.x, node.y, node.layer);
                    break;
                }
                node = came_from_[node.layer][node.x][node.y];
            }
            /********* 检查path的有效性 *********/
            nav_msgs::msg::Path valid_path;
            valid_path.header.frame_id = "map";
            valid_path.header.stamp = clock_->now(); // 必须设置有效时间戳

            for (const auto& n : path_nodes) {
                auto [x, y] = gridToWorld(n.x, n.y);

                // 校验坐标有效性
                if (std::isnan(x) || std::isnan(y)) {
                    RCLCPP_ERROR(rclcpp::get_logger("planner"),
                                "Invalid gridToWorld conversion: (%d,%d)->(%.2f,%.2f)",
                                n.x, n.y, x, y);
                    continue;
                }

                geometry_msgs::msg::PoseStamped pose;
                pose.header = valid_path.header;

                // 确保高度有效
                float z = tomography_->getSimplifiedHeightLayers()[n.layer][n.x][n.y];
                if (std::isnan(z)) z = 0.0; // 安全回退

                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = z;
                pose.pose.orientation.w = 1.0; // 必须设置有效四元数

                valid_path.poses.push_back(pose);
            }

            RCLCPP_INFO(rclcpp::get_logger("planner"),
                       "Published VALID path with %zu points", valid_path.poses.size());
            return valid_path;
        }

        // 情况2：继续搜索,Check 8-connected neighborhood
        closed_set[current.layer][current.x][current.y] = true; // 标记已访问
        for (int dx = -1; dx <= 1; ++dx) { // 水平方向遍历8邻域
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx = current.x + dx;
                int ny = current.y + dy;

                // Boundary checks
                if (nx < 0 || nx >= tomography_->getMapDimX() ||
                    ny < 0 || ny >= tomography_->getMapDimY()) {
                    continue;
                }

                // Check possible layer transitions // 垂直方向遍历层级，允许上下跳一层
                for (int layer_offset = -1; layer_offset <= 1; ++layer_offset) {
                    int nl = current.layer + layer_offset;
                    if (nl < 0 || nl >= tomography_->getNumSimplifiedLayers()) {
                        continue;
                    }

                    // Skip if not traversable
                    if (!isTraversable(nx, ny, nl)) {
                        continue;
                    }

                    // Calculate movement cost
                    float movement_cost = sqrt(dx*dx + dy*dy) * tomography_->getResolution();

                    // Calculate height change cost
                    float height_diff = std::abs(
                        tomography_->getSimplifiedHeightLayers()[nl][nx][ny] -
                        current.height);
                    float height_cost = height_diff * cfg_.height_change_weight;

                    // Additional cost for layer changes
                    float layer_change_cost = (layer_offset != 0) ? cfg_.layer_change_penalty : 0.0f;

                    // Get terrain cost from tomography
                    float terrain_cost = tomography_->getSimplifiedCostLayers()[nl][nx][ny];

                    // Total cost
                    float tentative_g = current.g + movement_cost + height_cost +
                                      layer_change_cost + terrain_cost;

                    // Skip if we already have a better path
                    // 检查这个邻居节点是否已经被完全处理过
                    if (closed_set[nl].count(nx) && closed_set[nl][nx].count(ny)) {
                        continue;
                    }

                    // 验证从当前节点到邻居节点的移动是否符合机器人的物理限制
                    if (!isValidTransition(current, nx, ny, nl)) {
                        continue;
                    }

                    if (!g_score[nl].count(nx) || !g_score[nl][nx].count(ny) ||
                        tentative_g < g_score[nl][nx][ny]) {

                        // Create new node
                        path_node neighbor{
                            nx, ny, nl,
                            tomography_->getSimplifiedHeightLayers()[nl][nx][ny],
                            tentative_g,
                            heuristic(nx, ny, nl, end_x, end_y, end_layer)
                        };

                        // Update path tracking
                        came_from_[nl][nx][ny] = current;
                        g_score[nl][nx][ny] = tentative_g;
                        open_set.push(neighbor);
                    }
                }
            }
        }
    }

    // 情况3：搜索失败
    RCLCPP_WARN(rclcpp::get_logger("planner"), "No valid path found");
    return path;
}

/**
 * @brief 根据二维的起始点和终点，找到所有层中start和end点代价最小的层
 * @note 这种设计非常不合理，应当由机器人所在位置的高度决定起始层，由用户指定的目标高度决定终点层
 */
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

        if (end_cost < min_end_cos  t && isTraversable(end_x, end_y, l)) {
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

    RCLCPP_DEBUG(logger_, "Grid(%d,%d) -> World(%.2f,%.2f)", i, j, x, y);
    return {x, y};
}

float Planner::heuristic(int x1, int y1, int z1, int x2, int y2, int z2) const {
    // 3D Euclidean distance
    float dx = (x1-x2) * tomography_->getResolution();
    float dy = (y1-y2) * tomography_->getResolution();
    float dz = (z1-z2) * tomography_->getConfig().slice_dh;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// 未测试
bool Planner::isTraversable(int x, int y, int layer) const {
    if (layer < 0 || layer >= tomography_->getNumSimplifiedLayers() ||
        x < 0 || x >= tomography_->getMapDimX() ||
        y < 0 || y >= tomography_->getMapDimY()) {
        return false;
    }

    const auto& cost_layer = tomography_->getSimplifiedCostLayers()[layer];

    // Check if the point is invalid (NaN) or has infinite cost
    if (std::isnan(cost_layer[x][y]) ||
        cost_layer[x][y] >= tomography_->FLOAT_INFINITY) {
        return false;
    }

    // Additional checks for physical constraints
    return cost_layer[x][y] < this->cfg_.traversal_cost_threshold ;
}

// 自己加的
// 为四足设置的物理检查 未做代码适配
bool Planner::isValidHeightChange(float from_height, float to_height, float horizontal_dist) const {
    float height_diff = std::abs(to_height - from_height);

    // Changed to use this->cfg_
    if (height_diff > this->cfg_.max_step_height) {
        return false;
    }

    if (horizontal_dist > 0) {
        float slope = std::atan2(height_diff, horizontal_dist) * 180.0f / M_PI;
        if (slope > this->cfg_.max_slope) {
            return false;
        }
    }

    return true;
}

bool Planner::isValidTransition(const path_node& from, int to_x, int to_y, int to_layer) const {
    // Get heights
    float from_height = from.height;
    float to_height = tomography_->getSimplifiedHeightLayers()[to_layer][to_x][to_y];

    // Calculate horizontal distance
    float dx = (to_x - from.x) * tomography_->getResolution();
    float dy = (to_y - from.y) * tomography_->getResolution();
    float horizontal_dist = std::sqrt(dx*dx + dy*dy);

    return isValidHeightChange(from_height, to_height, horizontal_dist);
}
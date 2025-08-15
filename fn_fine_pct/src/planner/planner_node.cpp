#include "fn_fine_pct/Planner.hpp"
#include "fn_fine_pct/Tomography.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

/**
 * New Terminal:
 * source /opt/ros/humble/setup.bash
 * =================================
 * Enter Workspace:
 * colcon build
 * source install/setup.bash
 * =================================
 * ros2 run fn_fine_pct planner_node
 * =================================
 * ros2 run rviz2 rviz2
 * =================================
 * pcl_viewer building.pcd
 */

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("planner_node") {
        // 初始化 tomography（确保处理完成）
        tomography_ = std::make_shared<Tomography>();

        // 等待直到tomography处理完成
        while (!tomography_->isProcessingComplete()) {  // 需要在Tomography类中添加此方法
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            RCLCPP_INFO(this->get_logger(), "Waiting for tomography processing...");
        }

        // 初始化 planner
        planner_ = std::make_unique<Planner>(tomography_);

        // 创建路径发布器
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("pct_path", 10);

        // 创建可视化标记发布器
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_markers", 10);

        // 定时器触发路径规划（1秒后执行）
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                planAndPublishPath();
                // 单次执行后取消定时器
                timer_->cancel();
                // 可选：完成后关闭节点
                // rclcpp::shutdown();
            });
    }

    void planAndPublishPath() {
        // 设置起点和终点
        std::array<float, 2> start = {5.0f, 5.0f};  // 示例起点
        std::array<float, 2> end = {-6.0f, -1.0f};  // 示例终点

        // path planning --------------------------
        auto path = planner_->planPath(start, end);

        // 发布路径
        path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Published path with %zu points", path.poses.size());

        // 发布可视化标记
        publishPathMarkers(path);

        RCLCPP_INFO(this->get_logger(), "END::Planner Path through A* ============================================");
    }

    void publishPathMarkers(const nav_msgs::msg::Path& path) {
        // 检查路径是否为空
        if (path.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot publish markers for empty path");
            return;
        }

        // 起点标记
        visualization_msgs::msg::Marker start_marker;
        start_marker.header = path.header;
        start_marker.ns = "path_points";
        start_marker.id = 0;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        start_marker.pose = path.poses.front().pose;
        start_marker.scale.x = 0.3;
        start_marker.scale.y = 0.3;
        start_marker.scale.z = 0.3;
        start_marker.color.r = 0.0;
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;
        marker_pub_->publish(start_marker);

        // 终点标记
        visualization_msgs::msg::Marker end_marker = start_marker;
        end_marker.id = 1;
        end_marker.pose = path.poses.back().pose;
        end_marker.color.r = 1.0;
        end_marker.color.g = 0.0;
        marker_pub_->publish(end_marker);

        // 路径线标记
        visualization_msgs::msg::Marker line_marker;
        line_marker.header = path.header;
        line_marker.ns = "path_line";
        line_marker.id = 2;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.1;
        line_marker.color.r = 0.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 1.0;

        for (const auto& pose : path.poses) {
            line_marker.points.push_back(pose.pose.position);
        }
        marker_pub_->publish(line_marker);
    }

private:
    std::shared_ptr<Tomography> tomography_;
    std::unique_ptr<Planner> planner_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
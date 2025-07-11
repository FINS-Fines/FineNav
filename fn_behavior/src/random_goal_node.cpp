#include "fn_behavior/random_goal_node.hpp"
#include <random>
#include <sstream>
#include <algorithm>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fn_behavior
{
    RandomGoalAction::RandomGoalAction()
    : BT::SyncActionNode("", {})
    {
        goal_publisher_ = rclcpp::Node::make_shared("random_goal_publisher")->
                          create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    }

    RandomGoalAction::RandomGoalAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
        goal_publisher_ = rclcpp::Node::make_shared("random_goal_publisher")->
                          create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    }

    BT::PortsList RandomGoalAction::providedPorts()
    {
        return {
            BT::InputPort<std::string>("area", "目标区域范围（格式：[(x1,y1), (x2,y2), ...]）"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "生成的随机目标点")
        };
    }

    BT::NodeStatus RandomGoalAction::tick()
    {
        // RCLCPP_INFO(rclcpp::get_logger("RandomGoalAction"), "随机目标节点被触发，开始生成随机目标点");

        std::string area_str;
        if (!getInput("area", area_str)) {
            // RCLCPP_ERROR(rclcpp::get_logger("RandomGoalAction"), "未获取到区域参数 'area'");
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::Polygon area = parseArea(area_str);
        if (area.points.size() < 2) {
            // RCLCPP_ERROR(rclcpp::get_logger("RandomGoalAction"), "解析的区域无效（至少需要2个点），实际点数：%zu", area.points.size());
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::PoseStamped new_goal = generateRandomGoal(area);
        setOutput("output_goal", new_goal);

        if (goal_publisher_) {
            goal_publisher_->publish(new_goal);
            // RCLCPP_INFO(rclcpp::get_logger("RandomGoalAction"), "已发布目标位姿到 /goal_pose 主题");
        } else {
            // RCLCPP_WARN(rclcpp::get_logger("RandomGoalAction"), "发布者未正确初始化，无法发布目标位姿");
        }

        // RCLCPP_INFO(rclcpp::get_logger("RandomGoalAction"), "生成随机目标点: (%.2f, %.2f)",
        //             new_goal.pose.position.x, new_goal.pose.position.y);

        return BT::NodeStatus::SUCCESS;
    }

    geometry_msgs::msg::PoseStamped RandomGoalAction::generateRandomGoal(const geometry_msgs::msg::Polygon& area)
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = rclcpp::Clock().now();

        float min_x = area.points[0].x;
        float max_x = area.points[0].x;
        float min_y = area.points[0].y;
        float max_y = area.points[0].y;

        for (const auto& pt : area.points) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }

        constexpr float STEP_SIZE = 0.1; 

        int min_x_idx = static_cast<int>(std::floor(min_x / STEP_SIZE));
        int max_x_idx = static_cast<int>(std::ceil(max_x / STEP_SIZE));
        int min_y_idx = static_cast<int>(std::floor(min_y / STEP_SIZE));
        int max_y_idx = static_cast<int>(std::ceil(max_y / STEP_SIZE));


        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> dist_x(min_x_idx, max_x_idx);
        std::uniform_int_distribution<> dist_y(min_y_idx, max_y_idx);

        int x_idx = dist_x(gen);
        int y_idx = dist_y(gen);

        goal.pose.position.x = static_cast<float>(x_idx) * STEP_SIZE;
        goal.pose.position.y = static_cast<float>(y_idx) * STEP_SIZE;
        goal.pose.position.z = 0.0;

        std::uniform_real_distribution<> dist_yaw(0, 2 * M_PI);
        double yaw = dist_yaw(gen);
        goal.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));

        return goal;
    }

    geometry_msgs::msg::Polygon RandomGoalAction::parseArea(const std::string& area_str)
    {
        geometry_msgs::msg::Polygon area;
        std::string str = area_str;
        str.erase(std::remove_if(str.begin(), str.end(), ::isspace), str.end());
        std::string content = str.substr(1, str.size() - 2);

        size_t pos = 0;
        std::string delimiter = "),(";
        while ((pos = content.find(delimiter)) != std::string::npos) {
            std::string token = content.substr(0, pos);
            if (!token.empty()) {
                parseSinglePoint(token, area);
            }
            content.erase(0, pos + delimiter.length());
        }

        if (!content.empty()) {
            parseSinglePoint(content, area);
        }

        return area;
    }

    void RandomGoalAction::parseSinglePoint(const std::string& token, geometry_msgs::msg::Polygon& area)
    {
        std::string point_str = token;
        if (!point_str.empty() && point_str.front() == '(') {
            point_str = point_str.substr(1);
        }
        if (!point_str.empty() && point_str.back() == ')') {
            point_str = point_str.substr(0, point_str.size() - 1);
        }

        std::istringstream pt_ss(point_str);
        std::string x_str, y_str;
        if (std::getline(pt_ss, x_str, ',') && std::getline(pt_ss, y_str)) {
            try {
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                geometry_msgs::msg::Point32 pt;
                pt.x = x;
                pt.y = y;
                pt.z = 0.0;
                area.points.push_back(pt);
            } catch (const std::exception& e) {
            }
        }
    }

}  // namespace fn_behavior

void register_fine_nav2d_nodes(BT::BehaviorTreeFactory& factory)
{
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<fn_behavior::RandomGoalAction>(name, config);
    };
    factory.registerBuilder<fn_behavior::RandomGoalAction>("RandomGoalAction", builder);
}

PLUGINLIB_EXPORT_CLASS(fn_behavior::RandomGoalAction, BT::ActionNodeBase)

BT_REGISTER_NODES(factory)
{
    register_fine_nav2d_nodes(factory);
}
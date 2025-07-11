#ifndef FN_BEHAVIOR__RANDOM_GOAL_NODE_HPP_
#define FN_BEHAVIOR__RANDOM_GOAL_NODE_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace fn_behavior
{
class RandomGoalAction : public BT::SyncActionNode
{
public:
    RandomGoalAction();
    RandomGoalAction(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    geometry_msgs::msg::PoseStamped generateRandomGoal(const geometry_msgs::msg::Polygon& area);
    geometry_msgs::msg::Polygon parseArea(const std::string& area_str);
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    void parseSinglePoint(const std::string& token, geometry_msgs::msg::Polygon& area);
};

}  // namespace fn_behavior

#endif  // FN_BEHAVIOR__RANDOM_GOAL_NODE_HPP_
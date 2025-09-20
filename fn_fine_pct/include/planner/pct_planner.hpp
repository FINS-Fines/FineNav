// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_PCT_PLANNER_HPP
#define FINENAV2D_PCT_PLANNER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iomanip>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "Tomography.hpp"
#include "a_star.hpp"
#include "tf2_ros/buffer.h"          // 包含tf2_ros::Buffer的定义
#include "tf2_ros/transform_listener.h"  // 包含tf2_ros::TransformListener的定义
#include "tf2_ros/transform_broadcaster.h"  // 包含tf2_ros::TransformBroadcaster的定义
#include "rclcpp/duration.hpp"       // 用于TF查询的超时设置

#include <octomap/HeightOcTree.h>
#include "octomap_server.hpp"
#include <queue>

using namespace finenav_2d;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using FollowPath = nav2_msgs::action::FollowPath;
using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;

// 明确声明Action相关类型
using ComputePathServer = rclcpp_action::Server<ComputePathToPose>;
using ComputePathGoalHandle = rclcpp_action::ServerGoalHandle<ComputePathToPose>;
using ComputePathServer2 = rclcpp_action::Server<ComputePathThroughPoses>;

class PctPlanner : public rclcpp::Node {
  public:
    explicit PctPlanner(const rclcpp::NodeOptions& options);

  private:
    void initPlanner() const;
    void publishTomography() const;

    // 修正Action回调函数的返回类型
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid, 
        std::shared_ptr<const ComputePathToPose::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<ComputePathGoalHandle> goal_handle);
    
    void handle_accepted(const std::shared_ptr<ComputePathGoalHandle> goal_handle);
    
    void execute(const std::shared_ptr<ComputePathGoalHandle> goal_handle);

    std::string pcd_file_path_;
    bool tomography_visualize_;
    bool path_visualize_ = true;
    TomographyConfig tomography_config;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                // TF缓存
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   // TF监听器
    Eigen::Vector3d robot_current_position_ = Eigen::Vector3d::Zero();  // 机器人当前位置

    std::unique_ptr<Tomography> tomography_;
    std::unique_ptr<Astar> path_finder_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tomography_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    std::shared_ptr<ComputePathServer> compute_path_server_;
    std::shared_ptr<ComputePathServer2> compute_path_server_2; // 虚假的服务器

    // void update_robot_position(); 
    bool get_robot_position_manual(Eigen::Vector3d& position);

};

// 1.2 辅助结构：路径段（起点/终点索引）
struct PathSegment {
    unsigned int start;
    unsigned int end;
};

// 修复：使用空捕获列表，因为该函数不依赖外部变量
auto getFieldByDim = [](const geometry_msgs::msg::PoseStamped &msg, const unsigned int &dim) -> double {
    if (dim == 0) return msg.pose.position.x;
    else if (dim == 1) return msg.pose.position.y;
    else return msg.pose.position.z;
};

auto setFieldByDim = [](geometry_msgs::msg::PoseStamped &msg, const unsigned int dim, const double &value) {
    if (dim == 0) msg.pose.position.x = value;
    else if (dim == 1) msg.pose.position.y = value;
    else msg.pose.position.z = value;
};

auto updateApproximatePathOrientations = [](nav_msgs::msg::Path &path, bool &reversing_segment) {
    reversing_segment = false;
    if (path.poses.size() < 2) return;

    for (size_t i = 0; i < path.poses.size() - 1; ++i) {
        double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        double yaw = atan2(dy, dx);

        geometry_msgs::msg::Quaternion q;
        q.w = cos(yaw / 2.0);
        q.z = sin(yaw / 2.0);
        path.poses[i].pose.orientation = q;
    }
    path.poses.back().pose.orientation = path.poses[path.poses.size()-2].pose.orientation;
};

// 关键修复：将 [&] 改为 []，因为此lambda不捕获任何外部变量
auto findDirectionalPathSegments = [](const nav_msgs::msg::Path &path) -> std::vector<PathSegment> {
    std::vector<PathSegment> segments;
    if (path.poses.size() < 2) return segments;

    unsigned int segment_start = 0;
    double prev_dx = path.poses[1].pose.position.x - path.poses[0].pose.position.x;
    double prev_dy = path.poses[1].pose.position.y - path.poses[0].pose.position.y;

    for (unsigned int i = 2; i < path.poses.size(); ++i) {
        double curr_dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
        double curr_dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
        double dot_product = prev_dx * curr_dx + prev_dy * curr_dy;

        if (dot_product < 1e-6) {
            segments.push_back({segment_start, i-1});
            segment_start = i-1;
            prev_dx = curr_dx;
            prev_dy = curr_dy;
        }
    }
    segments.push_back({segment_start, static_cast<unsigned int>(path.poses.size()-1)});
    return segments;
};

#endif  // FINENAV2D_PCT_PLANNER_HPP
    
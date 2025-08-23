// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_PCT_PLANNER_HPP
#define FINENAV2D_PCT_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "Tomography.hpp"
#include "a_star.hpp"

using namespace finenav_2d;

class PctPlanner : public rclcpp::Node {
public:
    explicit PctPlanner(const rclcpp::NodeOptions& options);

    //TODO: 回调，接收目标位姿，启动A*规划，发布规划后的path


private:
    void initPlanner() const;

    void publishTomography() const;

    void goalCallback(const geometry_msgs::msg::PoseStamped& msg);

    std::string pcd_file_path_;
    bool tomography_visualize_;


    std::unique_ptr<Tomography> tomography_; // TODO：暂时设置，tomography完后销毁
    std::unique_ptr<Astar> path_finder_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tomography_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    // 可视化地图


};

#endif //FINENAV2D_PCT_PLANNER_HPP

// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_MAP_MANAGER_H
#define FINENAV2D_MAP_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <message_filters/subscriber.h>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Core>
#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap_msgs/srv/bounding_box_query.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.hpp"

#include "grid_map.hpp"
#include "octomap_server.hpp"
#include "terrain_analyzer_base.hpp"
#include "cloud_publish_helper.hpp"

namespace finenav_2d {

class MapManager : public rclcpp::Node {
public:
    explicit MapManager(const rclcpp::NodeOptions& options);
    ~MapManager();
    /**
    * @brief 发布局部地图
    */
    void publishLocalMap(const rclcpp::Time& stamp);
    void publishLocalcostMap();
    void AnalyzerInit();
    void publishBinaryOctoMap(const rclcpp::Time& rostime) const;
    void publishFullOctoMap(const rclcpp::Time& rostime) const;

private:
    /**
     * @brief 管理主流程
     */
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);


    // MapManager需要管理全局地图和代价地图，这里通过依赖注入的方式实现
    std::shared_ptr<GridMap<float>>local_map_;
    std::shared_ptr<OctoMapServer> global_map_;
    std::unique_ptr<pluginlib::ClassLoader<TerrainAnalyzerBase>> terrain_analyzer_loader_; // 插件加载器需要声明在管理的动态类之前
    std::shared_ptr<TerrainAnalyzerBase> terrain_analyzer_;
    TerrainAnalyzerInterface::Ptr terrain_analyzer_interface_;
    Eigen::ArrayXXf passability_array_;
    Eigen::ArrayXXf ground_array_;
    // Input1: 监听tf，map，base_link

    // Input2: pcd_cbk


    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf2_filter_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;    // 发布局部地图的点云消息
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;      // 发布分析后的ground信息
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;      // 发布分析后的ground信息
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr localcost_map_pub_; // 发布局部代价地图
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pub_; // 发布局部地图的点云消息
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_map_pub_;
    rclcpp::Publisher< octomap_msgs::msg::Octomap>::SharedPtr full_map_pub_;

    finenav_utils::CloudPublishHelper cloud_pub_helper_;


};

}
#endif //FINENAV2D_MAP_MANAGER_H
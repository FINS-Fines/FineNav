// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_MAP_MANAGER_H
#define FINENAV2D_MAP_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_ros/create_timer_ros.h>




#include "grid_map.hpp"


namespace finenav_2d {

class MapManager : public rclcpp::Node {
public:
    explicit MapManager(const rclcpp::NodeOptions& options);
    /**
    * @brief 发布局部地图
    */
    void publishLocalMap();

private:




    /**
     * @brief 管理主流程
     */
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);





    // MapManager需要管理全局地图和代价地图，这里通过依赖注入的方式实现
    GridMap<uint8_t> local_map_; // TODO: 后续改为依赖注入


    // Input1: 监听tf，map，base_link

    // Input2: pcd_cbk


    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2>  point_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> > tf2_filter_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;  // 发布局部地图的点云消息
};

}

#endif //FINENAV2D_MAP_MANAGER_H

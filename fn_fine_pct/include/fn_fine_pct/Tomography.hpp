#pragma once
#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "Config.hpp"
#include <string>

class Tomography : public rclcpp::Node {
public:
    Tomography() ;

private:
    void loadPCD();
    void processPointCloud();

    // 配置参数
    std::string pcd_file_path_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    // ROS 2组件
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};
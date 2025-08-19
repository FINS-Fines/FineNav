// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV2D_TOMOGRAPHY_HPP
#define FINENAV2D_TOMOGRAPHY_HPP

#include <string>
#include <vector>
#include <limits>
#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "type_defs.hpp"

class Tomography : public rclcpp::Node {
public:
    Tomography() ;

    int getMapDimX() const { return map_dim_x_; }
    int getMapDimY() const { return map_dim_y_; }
    float getResolution() const { return cfg_.resolution; }
    const std::vector<float>& getCenter() const { return center_; }

    // 添加常量定义
    static constexpr inline float FLOAT_INFINITY = std::numeric_limits<float>::infinity();
    static constexpr int8_t OCCUPIED = 100; // 完全障碍的整数值
    static constexpr int8_t FREE = 0;       // 自由空间的整数值

private:
    void loadPCD();
    void processPointCloud();

    // Algorithm functions
    void initMappingEnv();
    void clearMap();
    void point2map(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void computeGradients();
    void computeTraversability();
    void inflateCosts();
    void simplifyLayers();
    void publishStaticTransform();
    void publishCostmapAndGradients();

    // PCD文件路径
    std::string pcd_file_path_;

    // Map data
    std::vector<float> center_;
    int map_dim_x_;
    int map_dim_y_;
    int n_slice_init_;
    float slice_h0_;

    // Map layers
    // 输出的东西
    std::vector<TomographyLayer> layers_;

    // 内部暂存的东西
    std::vector<Layer> grad_mag_sq_;  // Gradient magnitude squared
    std::vector<Layer> grad_mag_max_;  // Max gradient component
    std::vector<Layer>  trav_cost_;     // Traversability cost
    std::vector<Layer> inflated_cost_; // Inflated cost

    // Simplified layers
    std::vector<int> idx_simp_;

    // Inflation table
    std::vector<std::vector<float>> inf_table_; // 离线存储的膨胀表

    // point_cloud_config
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    // 发布
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_tomography_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ceiling_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_gradients_;

    // 添加发布方法
    TomographyConfig cfg_;  // Configuration parameters
    void publishTomographyResults();

    // ROS 2组件
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // FINENAV2D_TOMOGRAPHY_HPP
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "TomographyConfig.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

struct CostmapLayer {
    std::vector<std::vector<float>> costs;
    float min_height;
    float max_height;
};

class Tomography : public rclcpp::Node {
public:
    Tomography() ;

    // communicate to planner
    float getGroundHeight(int layer, int x, int y) const;
    int getMapDimX() const { return map_dim_x_; }
    int getMapDimY() const { return map_dim_y_; }
    const std::vector<float>& getCenter() const { return center_; }
    float getResolution() const { return cfg_.resolution; }
    float getInflatedCost(int layer, int x, int y) const;
    const TomographyConfig::Params& getConfig() const { return cfg_; }
    bool isProcessingComplete() const { return processing_complete_;  /* 需要在处理完成后设置此标志 */ }
    int getNumLayers() const { return layers_g_simp_.size(); }

    // 添加获取简化层数据的方法
    const std::vector<std::vector<std::vector<float>>>& getSimplifiedCostLayers() const {
        return layers_t_simp_;
    }
    const std::vector<std::vector<std::vector<float>>>& getSimplifiedHeightLayers() const {
        return layers_g_simp_;
    }
    int getNumSimplifiedLayers() const {
        return idx_simp_.size();
    }

    // 添加常量定义
    static constexpr inline float FLOAT_INFINITY = std::numeric_limits<float>::infinity();
    static constexpr int8_t OCCUPIED = 100; // 完全障碍的整数值
    static constexpr int8_t FREE = 0;       // 自由空间的整数值

    const std::vector<CostmapLayer>& getAllCostmapLayers() const {
        return costmap_layers_;
    }
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
    // 三个维度，[层索引][X坐标][Y坐标]
    std::vector<std::vector<std::vector<float>>> layers_g_;  // Ground layers
    std::vector<std::vector<std::vector<float>>> layers_c_;  // Ceiling layers
    std::vector<std::vector<std::vector<float>>> grad_mag_sq_;  // Gradient magnitude squared
    std::vector<std::vector<std::vector<float>>> grad_mag_max_;  // Max gradient component
    std::vector<std::vector<std::vector<float>>> trav_cost_;     // Traversability cost
    std::vector<std::vector<std::vector<float>>> inflated_cost_; // Inflated cost

    // Simplified layers
    std::vector<int> idx_simp_;
    std::vector<std::vector<std::vector<float>>> layers_t_simp_;
    std::vector<std::vector<std::vector<float>>> layers_g_simp_;
    std::vector<std::vector<std::vector<float>>> layers_c_simp_;
    std::vector<std::vector<std::vector<float>>> trav_grad_x_; // 简化后地图的代价变化梯度
    std::vector<std::vector<std::vector<float>>> trav_grad_y_; // 简化后地图的代价变化梯度

    // Inflation table
    std::vector<std::vector<float>> inf_table_;

    // point_cloud_config
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    std::vector<CostmapLayer> costmap_layers_;

    // 发布
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_tomography_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ceiling_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_gradients_;

    // 添加发布方法
    TomographyConfig::Params cfg_;  // Configuration parameters
    void publishTomographyResults();

    // ROS 2组件
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    bool processing_complete_ = false ;

    bool hasGroundBelow(int layer, int x, int y) const;
};
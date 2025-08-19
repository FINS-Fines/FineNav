// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once
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

/**
 *  @brief tomography算法参数配置
 *  @details 在Tomography初始化时从ROS2参数服务器读取
 */
struct TomographyConfig {
    float resolution = 0.1f;       // Map resolution (meters)
    float slice_dh = 0.5f;         // Height interval between slices // TODO: 它是如何影响的
    float ground_h = 0.0f;         // Ground height
    int half_kernel_size = 7;      // Traversal kernel half-size
    float interval_min = 0.50f;     // Minimum traversable interval
    float interval_free = 0.65f;    // Free space interval
    float slope_max = 0.36f;      // Maximum traversable slope (degrees)
    float step_max = 0.17f;         // Maximum step height
    float standable_ratio = 0.2f;  // Ratio of standable points required
    float cost_barrier = 50.0f;  // Cost for non-traversable areas
    float safe_margin = 0.4f;      // Safe margin around obstacles // TODO:硬安全边界?
    float inflation = 0.2f;        // Inflation radius // TODO:膨胀层？
};

using Layer = Eigen::MatrixXf;

/**
 *  @brief tomography算法输出的数据结构
 */
struct TomographyLayer {
    Layer trav_cost; // Traversability cost layer
    Layer trav_grad_x; // Traversability gradient in x direction
    Layer trav_grad_y; // Traversability gradient in y direction
    Layer ground; // ground layer
    Layer ceiling; // ceiling layer
};


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
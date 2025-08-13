#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "Config.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class Tomography : public rclcpp::Node {
public:
    Tomography() ;

private:
    void loadPCD();
    void processPointCloud();

    // Configuration parameters
    struct Config {
        float resolution = 0.1f;       // Map resolution in meters
        float slice_dh = 0.2f;         // Height interval between slices
        float ground_h = 0.0f;         // Ground height
        int half_kernel_size = 2;      // Half size of traversal kernel
        float interval_min = 0.1f;     // Minimum traversable interval
        float interval_free = 0.5f;    // Free space interval
        float slope_max = 30.0f;       // Maximum traversable slope (degrees)
        float step_max = 0.3f;         // Maximum step height
        float standable_ratio = 0.5f;  // Ratio of standable points required
        float cost_barrier = 1000.0f;  // Cost for non-traversable areas
        float safe_margin = 0.2f;      // Safe margin around obstacles
        float inflation = 0.3f;        // Inflation radius
    };

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

    // Map data
    std::vector<float> center_;
    int map_dim_x_;
    int map_dim_y_;
    int n_slice_init_;
    float slice_h0_;

    // Map layers
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
    std::vector<std::vector<std::vector<float>>> trav_grad_x_;
    std::vector<std::vector<std::vector<float>>> trav_grad_y_;

    // Inflation table
    std::vector<std::vector<float>> inf_table_;

    // Configuration
    Config cfg_;

    // point_cloud_config
    std::string pcd_file_path_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    // 发布
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_tomography_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ceiling_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_gradients_;

    // 添加发布方法
    void publishTomographyResults();

    // ROS 2组件
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "pct_planner.hpp"

PctPlanner::PctPlanner(const rclcpp::NodeOptions& options) : Node(options.arguments()[0], options) {
    /********* Parameters for PctPlanner *********/
    this->declare_parameter("pcd_file_path", "");
    this->declare_parameter("tomography_visualize_", false);
    pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();
    tomography_visualize_ = this->get_parameter("tomography_visualize_").as_bool();

    //TODO: FOR DEBUG
    pcd_file_path_ = "/home/huigg/Desktop/nav_ws/FineNav2D/pcd/building.pcd";
    tomography_visualize_ = true;

    /********* Parameters for Tomography *********/
    TomographyConfig tomography_config;
    this->declare_parameter("resolution", tomography_config.resolution);
    this->declare_parameter("slice_dh", tomography_config.slice_dh);
    this->declare_parameter("ground_h", tomography_config.ground_h);
    this->declare_parameter("half_kernel_size", tomography_config.half_kernel_size);
    this->declare_parameter("interval_min", tomography_config.interval_min);
    this->declare_parameter("interval_free", tomography_config.interval_free);
    this->declare_parameter("slope_max", tomography_config.slope_max);
    this->declare_parameter("step_max", tomography_config.step_max);
    this->declare_parameter("standable_ratio", tomography_config.standable_ratio);
    this->declare_parameter("cost_barrier", tomography_config.cost_barrier);
    this->declare_parameter("safe_margin", tomography_config.safe_margin);
    this->declare_parameter("inflation", tomography_config.inflation);

    tomography_config.resolution = this->get_parameter("resolution").as_double();
    tomography_config.slice_dh = this->get_parameter("slice_dh").as_double();
    tomography_config.ground_h = this->get_parameter("ground_h").as_double();
    tomography_config.half_kernel_size = this->get_parameter("half_kernel_size").as_int();
    tomography_config.interval_min = this->get_parameter("interval_min").as_double();
    tomography_config.interval_free = this->get_parameter("interval_free").as_double();
    tomography_config.slope_max = this->get_parameter("slope_max").as_double();
    tomography_config.step_max = this->get_parameter("step_max").as_double();
    tomography_config.standable_ratio = this->get_parameter("standable_ratio").as_double();
    tomography_config.cost_barrier = this->get_parameter("cost_barrier").as_double();
    tomography_config.safe_margin = this->get_parameter("safe_margin").as_double();
    tomography_config.inflation = this->get_parameter("inflation").as_double();

    tomography_ = std::make_unique<Tomography>(tomography_config);

    initPlanner();

    // TODO: topic name 参数
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose_3d", 10, std::bind(&PctPlanner::goalCallback, this, std::placeholders::_1));

    // 可视化tomography
    if (tomography_visualize_) {
        // 需要设置qos为latched
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        tomography_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tomography_layers", qos);
        publishTomography();
    }
}

void PctPlanner::initPlanner() const {
    // 加载PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud) == -1) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load PCD file: " << pcd_file_path_);
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded PCD file: " << pcd_file_path_ << " with " << cloud->size() << " points");

    // voxel滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素大小
    voxel_filter.filter(*cloud);
    RCLCPP_INFO_STREAM(this->get_logger(), "Filtered cloud has " << cloud->size() << " points");

    // 执行tomography流程
    tomography_->setInputCloud(cloud);
    tomography_->startAlgorithm();
    // auto tomography_layers = tomography_->getOutputLayers();
}

void PctPlanner::publishTomography() const {
    auto layers = tomography_->getOutputLayers();

    // 1. 创建带颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->reserve(tomography_->getMapDimX() * tomography_->getMapDimY() * layers.size());

    // 2. 填充点云数据（按高度着色）
    for (size_t k = 0; k < layers.size(); ++k) {
        // 分层着色
        float hue = static_cast<float>(k) / layers.size() * 360.0f;

        // 将HSV转换为RGB (H:0-360, S:1.0, V:1.0)
        float c = 1.0f;
        float x = c * (1.0f - fabs(fmod(hue / 60.0f, 2.0f) - 1.0f));
        float m = 0.0f;

        float r, g, b;
        if (hue < 60) {
            r = c; g = x; b = 0;
        } else if (hue < 120) {
            r = x; g = c; b = 0;
        } else if (hue < 180) {
            r = 0; g = c; b = x;
        } else if (hue < 240) {
            r = 0; g = x; b = c;
        } else if (hue < 300) {
            r = x; g = 0; b = c;
        } else {
            r = c; g = 0; b = x;
        }

        // 转换为0-255范围
        uint8_t R = static_cast<uint8_t>((r + m) * 255);
        uint8_t G = static_cast<uint8_t>((g + m) * 255);
        uint8_t B = static_cast<uint8_t>((b + m) * 255);

        for (int i = 0; i < tomography_->getMapDimX(); ++i) {
            for (int j = 0; j < tomography_->getMapDimY(); ++j) {
                if (std::isnan(layers[k].ground(i, j))) { continue; }

                pcl::PointXYZRGB point;
                // 坐标转换
                point.x = tomography_->getCenter()[0] + (i - tomography_->getMapDimX() / 2) * tomography_->getResolution();
                point.y = tomography_->getCenter()[1] + (j - tomography_->getMapDimY() / 2) * tomography_->getResolution();
                point.z = layers[k].ground(i, j);

                // set color option
                point.r = R;
                point.g = G;
                point.b = B;

                colored_cloud->push_back(point);
            }
        }
    }

    // 3. 转换并发布点云
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*colored_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map"; // 设置坐标系     // TODO: frame_id 参数
    cloud_msg.header.stamp = this->now();

    // TODO: 专门设置存储results的pcd
    const std::string pcd_path = "tomography_results.pcd";
    if (pcl::io::savePCDFileBinary(pcd_path, *colored_cloud) == 0) {
        RCLCPP_INFO(this->get_logger(), "Successfully saved to %s", pcd_path.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file!");
    }

    tomography_pub_->publish(cloud_msg);
}


void PctPlanner::goalCallback(const geometry_msgs::msg::PoseStamped& msg) {
    // TODO: 规划流程
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.arguments({"fn_global_map_node"});
    auto node = std::make_shared<PctPlanner>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

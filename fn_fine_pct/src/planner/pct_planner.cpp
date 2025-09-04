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
    pcd_file_path_ = "/home/fins/Desktop/Nav_ws/FineNav2D/fn_fine_pct/rsc/pcd/building.pcd";
    tomography_visualize_ = true;

    /********* Parameters for Tomography *********/
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
    path_finder_ = std::make_unique<Astar>();

    initPlanner();

    // TODO: topic name 参数
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", rclcpp::QoS(1).transient_local());
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose_3d", 10, std::bind(&PctPlanner::goalCallback, this, std::placeholders::_1));

    // 可视化tomography
    if (tomography_visualize_) {
        auto qos = rclcpp::QoS(1).transient_local();
        tomography_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tomography_layers", qos);
        publishTomography();
    }
}

void PctPlanner::initPlanner() const {
    // 加载PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud) == -1) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load PCD file: " << pcd_file_path_);
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Loaded PCD file: " << pcd_file_path_ << " with " << cloud->size() << " points");

    // voxel滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // 设置体素大小
    voxel_filter.filter(*cloud);
    RCLCPP_INFO_STREAM(this->get_logger(), "Filtered cloud has " << cloud->size() << " points");

    // 执行tomography流程
    tomography_->setInputCloud(cloud);
    tomography_->startAlgorithm();

    // 创建Astar
    auto layers = tomography_->getOutputLayers();
    int a_star_cost_threshold = 20;  // TODO: 参数
    int safe_cost_margin = 15;       // 用于轨迹后端优化 ，无用
    double step_cost_weight = 0.2;

    // TODO: 只需要三个参数，a_star_cost_threshold，layer, resolution
    path_finder_->initialize(a_star_cost_threshold, step_cost_weight, layers, HeuristicType::kDiagonal);
}

void PctPlanner::publishTomography() const {
    auto layers = tomography_->getOutputLayers();
    auto layers_num = layers.trav_cost.layers.size();

    // 1. 创建带颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->reserve(tomography_->getMapDimX() * tomography_->getMapDimY() * layers_num);

    // 2. 填充点云数据（按高度着色）
    for (size_t k = 0; k < layers_num; ++k) {
        // 分层着色
        float hue = static_cast<float>(k) / layers_num * 360.0f;

        // 将HSV转换为RGB (H:0-360, S:1.0, V:1.0)
        float c = 1.0f;
        float x = c * (1.0f - fabs(fmod(hue / 60.0f, 2.0f) - 1.0f));
        float m = 0.0f;

        float r, g, b;
        if (hue < 60) {
            r = c;
            g = x;
            b = 0;
        } else if (hue < 120) {
            r = x;
            g = c;
            b = 0;
        } else if (hue < 180) {
            r = 0;
            g = c;
            b = x;
        } else if (hue < 240) {
            r = 0;
            g = x;
            b = c;
        } else if (hue < 300) {
            r = x;
            g = 0;
            b = c;
        } else {
            r = c;
            g = 0;
            b = x;
        }

        // 转换为0-255范围
        uint8_t R = static_cast<uint8_t>((r + m) * 255);
        uint8_t G = static_cast<uint8_t>((g + m) * 255);
        uint8_t B = static_cast<uint8_t>((b + m) * 255);

        for (int x = 0; x < tomography_->getMapDimX(); ++x) {
            for (int y = 0; y < tomography_->getMapDimY(); ++y) {
                if (std::isnan(layers.ground(x, y, k))) {
                    continue;
                }

                pcl::PointXYZRGB point;
                // 坐标转换
                point.x =
                    tomography_->getCenter()[0] + (x - tomography_->getMapDimX() / 2) * tomography_->getResolution();
                point.y =
                    tomography_->getCenter()[1] + (y - tomography_->getMapDimY() / 2) * tomography_->getResolution();
                point.z = layers.ground(x, y, k);

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
    cloud_msg.header.frame_id = "map";  // 设置坐标系     // TODO: frame_id 参数
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
    RCLCPP_INFO(this->get_logger(), "Received new goal");
    // 我需要一个起点和终点

    path_finder_->search(Eigen::Vector3i(0, 50, 50), Eigen::Vector3i(1, 27, 52));

    auto path = path_finder_->getPath();

    // 发布路径
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";  // TODO: frame_id 参数
    path_msg.header.stamp = this->now();
    for (const auto& idx : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = static_cast<double>(idx[2]) * tomography_config.resolution ;
        pose.pose.position.y = static_cast<double>(idx[1]) * tomography_config.resolution ; 
        pose.pose.position.z = static_cast<double>(idx[0]) / 100.0; // TODO 传递高度信息的时候由于idx不得不为整数，保留两位小数
        pose.pose.orientation.w = 1.0;  // 无旋转
        printf("path point: [%.2f, %.2f, %.2f]\n", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        path_msg.poses.push_back(pose);
    }
    path_pub_->publish(path_msg);

    // 保存路径
    // std::ofstream ofs("/home/fins/Desktop/Nav_ws/FineNav2D/fn_fine_pct/rsc/pcd/planned_path.txt");
    // if (ofs.is_open()) {
    //     for (const auto& idx : path) {
    //         ofs << idx[0] << " " << idx[1] << " " << idx[2] << "\n";
    //     }
    //     ofs.close();
    //     RCLCPP_INFO(this->get_logger(), "Planned path saved to planned_path.txt");
    // } else {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to save planned path!");
    // }

    // TODO: 规划流程
    // layer 0, row 114, col 222, g 206.17
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

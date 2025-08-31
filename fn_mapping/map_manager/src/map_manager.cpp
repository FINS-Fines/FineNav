// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <chrono>
#include <Eigen/Core>

#include <pcl_ros/transforms.hpp>

#include "map_manager.h"

namespace finenav_2d {

using std::chrono_literals::operator"" ms;

MapManager::MapManager(const rclcpp::NodeOptions& options)
    : Node("map_manager", options) {
    RCLCPP_INFO(get_logger(), "MapManager initialized");

    local_map_ = std::make_shared<GridMap<float>>(Length{10.0, 10.0, 10.0}, 0.05);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(),
                                                  this->get_node_timers_interface()));
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    rclcpp::QoS qos = rclcpp::SensorDataQoS(); // 自动 BestEffort, Depth 10
    point_sub_.subscribe(this, "/lidar", qos.get_rmw_qos_profile());

    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        point_sub_, *tf2_buffer_, "/base_lidar", 10,
        this->get_node_logging_interface(), this->get_node_clock_interface(), 100ms);
    tf2_filter_->registerCallback(&MapManager::pointcloudCallback, this);

    // 初始化发布器
    local_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 10);
    localcost_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_cost_map", 10);
    test_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_test", 10);

    // 地形分析
    terrain_analyzer_loader_ = std::make_unique<pluginlib::ClassLoader<TerrainAnalyzerBase>>(
        "fn_terrain_analysis_core", "finenav_2d::TerrainAnalyzerBase");
    try {
        terrain_analyzer_ = terrain_analyzer_loader_->createSharedInstance("fn_terrain_analysis/SimpleAnalyzer");
        RCLCPP_INFO(get_logger(), "TerrainAnalyzer plugin loaded successfully");
    } catch (pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(get_logger(), "Failed to load TerrainAnalyzer plugin: %s", ex.what());
    }

    // TODO: 设置为LifeCycleNode，允许on_configure时配置terrain_analyzer_

}

void MapManager::AnalyzerInit() {
    passability_array_ = Eigen::ArrayXXf::Constant(local_map_->getSize().x(), local_map_->getSize().y(), 0);

    TerrainAnalyzerInterface::Getter gridmap_getter = [this](const size_t& x, const size_t& y) -> std::span<float> {
        return local_map_->getVoxelsAlongZ(static_cast<int>(x) - local_map_->getSize().x() / 2,
                                           static_cast<int>(y) - local_map_->getSize().y() / 2);
    };

    auto terrain_setter = [this](const size_t& idx_x, const size_t& idx_y, const float& value) {
        passability_array_(idx_x, idx_y) = value;
        if (value > 0) {
            std::cout << value << std::endl;
        }
    };
    // TODO: 目前只能支持将是否通行写出来，后续可以考虑是否要给出中间结果，例如ground和ceiling

    auto min_pt = local_map_->getPosition(local_map_->getMinIndex());
    auto map_size = local_map_->getSize();
    terrain_analyzer_interface_ = std::make_shared<TerrainAnalyzerInterface>(
        map_size.x(), map_size.y(), min_pt.x(), min_pt.y(),
        gridmap_getter, terrain_setter);

    terrain_analyzer_->configure(shared_from_this(), "terrain_analysis", terrain_analyzer_interface_);
    RCLCPP_INFO(get_logger(), "MapManager initialized successfully!");

}


void MapManager::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    geometry_msgs::msg::TransformStamped tf_base;

    // TODO: 获取map->base_link->base_lidar的变换
    try {
        tf_base = tf2_buffer_->lookupTransform(
            "base_link", // 目标坐标系
            "odom",      // 源坐标系
            msg->header.stamp,
            100ms
            );
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(*msg, pc);

    // 局部地图跟随移动到base_link处
    Position base_pos;
    base_pos.x() = tf_base.transform.translation.x;
    base_pos.y() = tf_base.transform.translation.y;
    base_pos.z() = tf_base.transform.translation.z;
    local_map_->moveTo(base_pos);

    // 点云数据旋转，与local_map对齐
    Eigen::Quaterniond rotation(
        tf_base.transform.rotation.w,
        tf_base.transform.rotation.x,
        tf_base.transform.rotation.y,
        tf_base.transform.rotation.z
        );

    auto sensor_to_base_rotation = Eigen::Affine3d::Identity();
    sensor_to_base_rotation.rotate(rotation.inverse()); // 只设置旋转，不设置平移
    pcl::transformPointCloud(pc, pc, sensor_to_base_rotation.matrix().cast<float>());

    // Raycasting
    auto t0 = std::chrono::high_resolution_clock::now();

    // 先清除所用路径上的栅格
    for (const auto& p : pc) {
        std::vector<Index> ray_indices;
        Position end{p.x, p.y, p.z};

        if (!local_map_->isInside(end)) {
            continue;
        }
        if (local_map_->rayCast(end, ray_indices)) {
            // 遍历光线上的栅格
            for (size_t i = 0; i + 1 < ray_indices.size(); ++i) { // TODO: 这里每次需要遍历一串栅格，效率较低
                local_map_->at(ray_indices[i]) = NAN; // TODO: 将空闲设置为宏定义
            }
        }
    }

    // 再将点云所在栅格设置为Occupied
    for (const auto& p : pc) {
        Position end{p.x, p.y, p.z};
        if (!local_map_->isInside(end)) {
            continue;
        }
        local_map_->atPosition(end) = p.z; // 设置为点的高度
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
    // RCLCPP_INFO(this->get_logger(), "Point cloud processing time: %ld ms", duration.count());
    // RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", points.size());

    // 发布局部地图（可视化）
    cloud_pub_helper_.configure(local_map_pub_, true, "base_link");
    for (auto it = local_map_->begin(); it != local_map_->end(); ++it) {
        Position pos = it.getPosition();
        if (!std::isnan(*it)) {
            // 将点云转换到base_link坐标系下
            Position pt_in_map = {pos.x(), pos.y(), *it};
            Position pt_in_base = rotation * pt_in_map;
            cloud_pub_helper_.addPoint(pt_in_base.x(), pt_in_base.y(), pt_in_base.z());
        }
    }
    cloud_pub_helper_.publish(this->now());

    // 地形分析
    if (terrain_analyzer_) {
        terrain_analyzer_->analyzeTerrain();
    }
    publishLocalcostMap();
}

void MapManager::publishLocalcostMap() {
    std::vector<Index> Ocuppied_cells;
    local_map_->selectCellsByCondition(Ocuppied_cells, [](const float& value) {
        return value == 1; // 假设 true 表示 Ocuppied
    });

    auto min_pt = local_map_->getPosition(local_map_->getMinIndex());

    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = "map"; //map的坐标系位置在哪里
    grid_msg.header.stamp = this->now();
    grid_msg.info.resolution = local_map_->getResolution();                                          // 地图分辨率
    grid_msg.info.width = local_map_->getSize().x();                                                 // 地图宽度
    grid_msg.info.height = local_map_->getSize().y();                                                // 地图高度
    grid_msg.info.origin.position.x = min_pt.x();
    grid_msg.info.origin.position.y = min_pt.y();
    grid_msg.info.origin.position.z = 0.0;

    grid_msg.data.resize(local_map_->getSize().x() * local_map_->getSize().y());
    for (size_t i = 0; i < local_map_->getSize().x() * local_map_->getSize().y(); ++i) {
        grid_msg.data[i] = passability_array_(i % static_cast<size_t>(local_map_->getSize().x()),
                                              i / static_cast<size_t>(local_map_->getSize().x())) * 100;
    }
    localcost_map_pub_->publish(grid_msg);
}
}
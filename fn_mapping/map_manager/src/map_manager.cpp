// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <chrono>
#include "map_manager.h"
#include <Eigen/Core>

namespace finenav_2d {
MapManager::MapManager(const rclcpp::NodeOptions& options)
    : Node("map_manager", options) {
    RCLCPP_INFO(get_logger(), "MapManager initialized");

    std::chrono::duration<int> buffer_timeout(1);

    local_map_ = std::make_shared<GridMap<float>>(Length{10.0, 10.0, 10.0}, 0.05);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(),
                                                  this->get_node_timers_interface()));

    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    rclcpp::QoS qos = rclcpp::SensorDataQoS();  // 自动 BestEffort, Depth 10
    point_sub_.subscribe(this, "/lidar", qos.get_rmw_qos_profile());

    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        point_sub_, *tf2_buffer_, "/base_lidar", 100, this->get_node_logging_interface(),
        this->get_node_clock_interface(), buffer_timeout);
    tf2_filter_->registerCallback(&MapManager::pointcloudCallback, this);


    // 初始化发布器
    local_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "local_map", 10  // 队列长度
    );
    localcost_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "local_cost_map", 10
        );
    RCLCPP_INFO(get_logger(), "Publishers initialized: local_map, terrain_test");


    // 地形分析
    terrain_analyzer_loader_ = std::make_unique<pluginlib::ClassLoader<TerrainAnalyzerBase>>("fn_terrain_analysis_core", "finenav_2d::TerrainAnalyzerBase");
    try {

        // 放到类的成员，注意下这是个共享指针
        terrain_analyzer_ = terrain_analyzer_loader_->createSharedInstance("fn_terrain_analysis/SimpleAnalyzer");
        RCLCPP_INFO(get_logger(), "TerrainAnalyzer plugin loaded successfully");
    }
    catch(pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(get_logger(), "Failed to load TerrainAnalyzer plugin: %s", ex.what());
    }

    passability_array_ = Eigen::ArrayXXf::Constant(local_map_->getSize().x() , local_map_->getSize().y(), 0);

    TerrainAnalyzerInterface::Getter gridmap_getter = [this](const size_t& x, const size_t& y) -> std::span<float> {
        return local_map_->getVoxelsAlongZ(static_cast<int>(x)-local_map_->getSize().x()/2, static_cast<int>(x)-local_map_->getSize().y()/2);
    };

    auto terrain_setter = [this](const size_t& idx_x, const size_t& idx_y, const float& value) {
        passability_array_(idx_x, idx_y) = value;
    };
    // TODO: 目前只能支持将是否通行写出来，后续可以考虑是否要给出中间结果，例如ground和ceiling

    terrain_analyzer_interface_ = std::make_shared<TerrainAnalyzerInterface>(
         local_map_->getSize().x(), local_map_->getSize().y(),
         local_map_->getOrigin().x()-local_map_->getSize().x()/2*local_map_->getResolution(),
         local_map_->getOrigin().y()-local_map_->getSize().y()/2*local_map_->getResolution(),
        gridmap_getter, terrain_setter);

    // 对应的对象的共享指针
    terrain_analyzer_->configure(shared_from_this(), "terrain_analysis", terrain_analyzer_interface_);
    RCLCPP_INFO(get_logger(), "MapManager initialized successfully!");
}


void MapManager::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // 改写成message_filters实现的tf接收，文档......
    // 参考octomap_server

    geometry_msgs::msg::TransformStamped tf_base;
    try {
        tf_base = tf2_buffer_->lookupTransform(
            "base_link",   // 目标坐标系
            "odom",      // 源坐标系
            msg->header.stamp,
            rclcpp::Duration::from_seconds(0.1)  // 超时 100ms
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
        return;
    }

    //move to base_link
    Position base_link_pos;
    base_link_pos.x() = tf_base.transform.translation.x;
    base_link_pos.y() = tf_base.transform.translation.y;
    base_link_pos.z() = tf_base.transform.translation.z;
    local_map_->moveTo(base_link_pos);

    std::vector<Eigen::Vector3d> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        points.emplace_back(*iter_x, *iter_y, *iter_z);
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    for (const auto& p : points) {
        std::vector<Index> ray_indices;
        Position end{p.x(), p.y(), p.z()};
        if(!local_map_->isInside(end)) {
            continue;
        }
        // 调用 rayCast
        if (local_map_->rayCast(end, ray_indices)) {
            // 遍历光线上的栅格
            for (size_t i = 0; i + 1 < ray_indices.size(); ++i) {
                local_map_->at(ray_indices[i]) = NAN;   // 清除，设置为空闲
            }
        }
    }

    for (const auto& p : points) {
        if(!local_map_->isInside(p)) {
            continue;
        }
        local_map_->atPosition(p) = p.z();
        // 打印出来，确认写入了什么
    }


    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
    RCLCPP_INFO(this->get_logger(), "Point cloud processing time: %ld ms", duration.count());
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", points.size());




    // 发布地图（可视化）
    publishLocalMap();

    // 地形分析
    if (terrain_analyzer_) {
        terrain_analyzer_->analyzeTerrain();
    }
    publishLocalcostMap();
}

void MapManager::publishLocalMap() {

    // 获取local_map_中所有Occupied的格子
    std::vector<Index> Ocuppied_cells;
    local_map_->selectCellsByCondition(Ocuppied_cells, [](const float& value) {
        return value!=-100; // 假设 true 表示 Ocuppied
    });

    //创建Pointcloud2信息
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "base_link";
    cloud.header.stamp = this->now();
    cloud.height = 1; // 非组织型点云
    cloud.width = Ocuppied_cells.size(); // 点的数量
    cloud.is_dense = true;
    cloud.is_bigendian = false;
    cloud.point_step = 3 * sizeof(float); // x, y, z
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);

    //设置field
    cloud.fields.resize(3);
    cloud.fields[0].name = "x"; cloud.fields[0].offset = 0; cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[0].count = 1;
    cloud.fields[1].name = "y"; cloud.fields[1].offset = sizeof(float); cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[1].count = 1;
    cloud.fields[2].name = "z"; cloud.fields[2].offset = 2*sizeof(float); cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[2].count = 1;


    // 3. 将Ocuppied_cells转换为点云
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const Index& idx : Ocuppied_cells) {
        Position pos;
        pos = local_map_->getPosition(idx); // 获取对应的世界坐标

        *iter_x = pos.x(); ++iter_x;
        *iter_y = pos.y(); ++iter_y;
        *iter_z = pos.z(); ++iter_z;
    }

    // 4. 发布点云
    local_map_pub_->publish(cloud);
}

void MapManager::publishLocalcostMap() {
    std::vector<Index> Ocuppied_cells;
    local_map_->selectCellsByCondition(Ocuppied_cells, [](const float& value) {
        return value ==1 ; // 假设 true 表示 Ocuppied
    });

    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = "map";   //map的坐标系位置在哪里
    grid_msg.header.stamp = this->now();
    grid_msg.info.resolution = local_map_->getResolution(); // 地图分辨率
    grid_msg.info.width = local_map_->getLength().x(); // 地图宽度
    grid_msg.info.height = local_map_->getLength().x(); // 地图高度
    grid_msg.info.origin.position.x = local_map_->getOrigin().x(); //应该设置成什么
    grid_msg.info.origin.position.y = local_map_->getOrigin().y();    //应该设置成什么
    grid_msg.info.origin.position.z = 0.0;

    grid_msg.data.resize(local_map_->getLength().x() * local_map_->getLength().y());
    for (size_t i = 0; i < local_map_->getLength().x() * local_map_->getLength().y(); ++i) {
        grid_msg.data[i] = passability_array_(i%static_cast<size_t>(local_map_->getLength().x()),i/static_cast<size_t>(local_map_->getLength().x()));
    }
    localcost_map_pub_->publish(grid_msg);
}
}
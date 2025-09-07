// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <chrono>
#include <Eigen/Core>
#include <pcl_ros/transforms.hpp>

#include "map_manager.h"

namespace finenav_2d {

#define NEW_OCCUPIED (std::numeric_limits<float>::infinity())

using std::chrono_literals::operator"" ms;            //Index比较器
struct Vector3iCompare {
    bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
        if (a.x() != b.x()) return a.x() < b.x();
        if (a.y() != b.y()) return a.y() < b.y();
        return a.z() < b.z();
    }
};

MapManager::MapManager(const rclcpp::NodeOptions& options)
    : Node("map_manager", options) {
    RCLCPP_INFO(get_logger(), "MapManager initialized");

    local_map_ = std::make_shared<GridMap<float>>(Length{5.0, 5.0, 5.0}, 0.05);
    global_map_ = std::make_shared<OctoMapServer>(0.05);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(),
                                                  this->get_node_timers_interface()));
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    rclcpp::QoS qos = rclcpp::SensorDataQoS(); // 自动 BestEffort, Depth 10
    point_sub_.subscribe(this, "/cloud_registered_body", qos.get_rmw_qos_profile());

    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        point_sub_, *tf2_buffer_, "/odom", 10,
        this->get_node_logging_interface(), this->get_node_clock_interface(), 100ms);
    tf2_filter_->registerCallback(&MapManager::pointcloudCallback, this);

    // 初始化发布器
    local_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 10);
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground", 10);
    localcost_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_cost_map", 10);
    test_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_test", 10);
    binary_map_pub_ = create_publisher<octomap_msgs::msg::Octomap>("octomap_binary", qos);

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
    ground_array_ = Eigen::ArrayXXf::Constant(local_map_->getSize().x(), local_map_->getSize().y(), NAN);

    TerrainAnalyzerInterface::Getter gridmap_getter = [this](const size_t& x, const size_t& y) -> std::span<float> {
        return local_map_->getVoxelsAlongZ(static_cast<int>(x) - local_map_->getSize().x() / 2,
                                           static_cast<int>(y) - local_map_->getSize().y() / 2);
    };

    auto terrain_setter = [this](const std::string& field, const size_t& idx_x, const size_t& idx_y, const float& value) {
        if (field == "traversablilty") {
            passability_array_(idx_x, idx_y) = value;
        } else if (field == "valid_ground") {
            ground_array_(idx_x, idx_y) = value;
        } else {
            RCLCPP_WARN(get_logger(), "Unknown field name: %s", field.c_str());
        }
    };
    // TODO: 目前只能支持将是否通行写出来，后续可以考虑是否要给出中间结果，例如ground和ceiling

    auto min_pt = local_map_->getPosition(local_map_->getMinIndex());
    auto map_size = local_map_->getSize();
    terrain_analyzer_interface_ = std::make_shared<TerrainAnalyzerInterface>(
        map_size.x(), map_size.y(),
        gridmap_getter, terrain_setter);

    terrain_analyzer_->configure(shared_from_this(), "terrain_analysis", terrain_analyzer_interface_);
    RCLCPP_INFO(get_logger(), "MapManager initialized successfully!");

}


void MapManager::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // 获取tf
    const std::string sensor_frame = "base_lidar";     // TODO: 这些作为参数给用户
    const std::string base_frame = "base_link";
    const std::string world_frame = "map";

    geometry_msgs::msg::TransformStamped base_to_world_transform_stamped;
    geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
    try {
        base_to_world_transform_stamped = tf2_buffer_->lookupTransform(world_frame, base_frame, msg->header.stamp, 100ms);
        sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(world_frame, sensor_frame, msg->header.stamp, 100ms);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
        return;
    }

    std::vector<Index> Octomap_indices;           //获取移动后需要处理的栅格索引
    std::map<Index, float,  Vector3iCompare> temporary_local_map;          //临时存储现在的需要处理的localmap信息
    Position base_posistion, sensor_position;
    base_posistion.x() = base_to_world_transform_stamped.transform.translation.x;
    base_posistion.y() = base_to_world_transform_stamped.transform.translation.y;
    base_posistion.z() = base_to_world_transform_stamped.transform.translation.z;
    sensor_position.x() = sensor_to_world_transform_stamped.transform.translation.x;
    sensor_position.y() = sensor_to_world_transform_stamped.transform.translation.y;
    sensor_position.z() = sensor_to_world_transform_stamped.transform.translation.z;

    //定义回调函数
    auto callback_in = [&](finenav_2d::OctoMapServer::IteratorBase* it) {              //将local_map_的信息读入global_map_
        std::cout<<"22222222222"<<std::endl;
        auto pt = it->getCoordinate();
        Index idx = local_map_->getIndex(Position(pt.x(),pt.y(),pt.z()));
        if(!std::isnan(temporary_local_map[idx])) {
			global_map_->getOctree().updateNode(pt, true);                    // 将Octomap中对应位置设置为占据
        }else{
            global_map_->getOctree().updateNode(pt, false);
        }
    };

    auto callback_out = [&](finenav_2d::OctoMapServer::IteratorBase* it) {             //将global_map_的信息读入local_map_
		auto tree = global_map_->getOctree();
        auto pt = it->getCoordinate();
        if (tree.isNodeOccupied(**it)) { // 如果为占据则发布
        	local_map_->atPosition(Position(pt.x(),pt.y(),pt.z()))= pt.z() ;
        }else{
            local_map_->atPosition(Position(pt.x(),pt.y(),pt.z())) = NAN ;
        }
    };

    OctoMapServer::Point moved_distance(base_posistion.x() - local_map_->getOrigin().x(),               //移动的向量
                         base_posistion.y() - local_map_->getOrigin().y(),
                         base_posistion.z() - local_map_->getOrigin().z());

    OctoMapServer::Point original_min(local_map_->getOrigin().x() - local_map_->getLength().x() / 2,         //移动前的地图下边界
                       local_map_->getOrigin().y() - local_map_->getLength().y() / 2,
                       local_map_->getOrigin().z() - local_map_->getLength().z() / 2);

    OctoMapServer::Point original_max(local_map_->getOrigin().x() + local_map_->getLength().x() / 2,         //移动前的地图上边界
                       local_map_->getOrigin().y() + local_map_->getLength().y() / 2,
                       local_map_->getOrigin().z() + local_map_->getLength().z() / 2);


    global_map_->traverseMoveDifferenceRegion(original_min, original_max, moved_distance, callback_out, false, OctoMapServer::MoveDifferenceMode::REMOVED);

    // 移动local_map_
    local_map_->moveTo(base_posistion , true , Octomap_indices);

    //临时存储
    for(Index idx : Octomap_indices) {
        temporary_local_map.insert({idx, local_map_->at(idx)});
    }
    //local_map_从global内获取信息
    global_map_->traverseMoveDifferenceRegion(original_min, original_max, moved_distance, callback_out, false, OctoMapServer::MoveDifferenceMode::REMOVED);

    //进行raycast
    // 将点云变换到世界坐标系
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(*msg, pc);
    pcl_ros::transformPointCloud(pc, pc, sensor_to_world_transform_stamped);

    // 根据raycasting向local_map_插入点云数据
    for (const auto& p : pc) { // 标记新增占据栅格
        Position end{p.x, p.y,p.z};
        if (!local_map_->isInside(end)) {
            continue;
        }
        local_map_->atPosition(end) = NEW_OCCUPIED;
    }
     for (const auto& p : pc) { // 执行raycasting
         std::vector<Index> ray_indices;
         Position end{p.x, p.y,p.z};
         if (!local_map_->isInside(end)) {
             continue;
         }

         //大致判断平面方向
         int Normal[3] = { 0 , 0 , 0 } ;   //x:0,y:1,z:2,点数稀疏，无效为-1
         Index end_index = local_map_->getIndex(end);
         int count_x = 0, count_y = 0 , count_z = 0;
         for(int i = -1 ; i < 2 ; i++){
             for(int j = -1 ; j < 2 ; j++){
                 Index neighbor = end_index + Index(0,i,j);
                 if(local_map_->isInside(neighbor)) {
				     if(!std::isnan(local_map_->at(neighbor))){
                     count_x++;
                     }
                 }
             }
         }

         for(int i = -1 ; i < 2 ; i++){
             for(int j = -1; j < 2 ; j++){
                 Index neighbor = end_index + Index(i,0,j);
                 if(local_map_->isInside(neighbor)) {
				     if(!std::isnan(local_map_->at(neighbor))){
                     count_y++;
                     }
                 }
             }
         }

         for(int i = -1 ; i < 2 ; i++){
             for(int j = -1 ; j < 2 ; j++){
                 Index neighbor = end_index + Index(i,j,0);
                 if(local_map_->isInside(neighbor)) {
				     if(!std::isnan(local_map_->at(neighbor))){
                     count_z++;
                     }
                 }
             }
         }
         if(count_z >= count_x && count_z >= count_y){
             Normal[2] = 1;
         }
         else if(count_y >= count_x && count_y >= count_z){
             Normal[1] = 1;
         }
         else if(count_x >= count_y && count_x >= count_z){
             Normal[0] = 1;
         }

         if (local_map_->rayCast(sensor_position, end, ray_indices)) {
             // 遍历光线上的栅格
             for (size_t i = 0; i + 1 < ray_indices.size(); ++i) {
                 if(local_map_->at(ray_indices[i])== NEW_OCCUPIED) { // 光线在遇到新增点截断
                     break;
                 }
                 if(Normal[0] == 1 && abs(ray_indices[i].x() - end_index.x()) <= 0) { // x平面
                     break;
                 }
                 if(Normal[1] == 1 && abs(ray_indices[i].y() - end_index.y()) <= 0) { // y平面
                     break;
                 }
                 if(Normal[2] == 1 && abs(ray_indices[i].z() - end_index.z()) <= 0 ) { // z平面
                     break;
                 }

                 local_map_->at(ray_indices[i]) = NAN; // 设置为Free
             }
         }
     }
    for (const auto& p : pc) { // 再将点云所在栅格设置为Occupied
        Position end{p.x, p.y,p.z};
        if (!local_map_->isInside(end)) {
            continue;
        }
        local_map_->atPosition(end) = p.z; // 栅格中存储点的实际高度
    }

    // auto t1 = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
    // RCLCPP_INFO(this->get_logger(), "Point cloud processing time: %ld ms", duration.count());
    // RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", points.size());

    // 发布局部地图（可视化）
    cloud_pub_helper_.configure(local_map_pub_, true, world_frame);
    for (auto it = local_map_->begin(); it != local_map_->end(); ++it) {
        Position pos = it.getPosition();
        if (!std::isnan(*it)) { // 如果栅格被占据，则发布
            cloud_pub_helper_.addPoint(pos.x(), pos.y(), local_map_->atPosition(pos));
        }
    }
    cloud_pub_helper_.publish(msg->header.stamp);

    // 地形分析
    if (terrain_analyzer_) {
        terrain_analyzer_->analyzeTerrain();
    }

    //发布ground分析结果
    cloud_pub_helper_.configure(ground_pub_, true, world_frame);
    for(int x = 0 ; x < local_map_->getSize().x() ; x++){
        for(int y = 0 ; y <local_map_->getSize().y() ; y++){
            if(!std::isnan(ground_array_(x,y))){
                Position pos = local_map_->getPosition(Index(x - local_map_->getSize().x() / 2,
                                                            y - local_map_->getSize().y() / 2,0));
                cloud_pub_helper_.addPoint(pos.x(), pos.y(), ground_array_(x,y),{255,0,0});
            }
        }
    }
    cloud_pub_helper_.publish(msg->header.stamp);

    publishLocalcostMap();

    //将local_map_出界的数据读入global_map_
    global_map_->traverseMoveDifferenceRegion(original_min, original_max, moved_distance, callback_in, true, OctoMapServer::MoveDifferenceMode::ADDED);
    publishBinaryOctoMap(msg->header.stamp);

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
    grid_msg.info.width = local_map_->getSize().x();                                               // 地图宽度
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

void MapManager::publishBinaryOctoMap(const rclcpp::Time& rostime) const {
    octomap_msgs::msg::Octomap map;
    map.header.frame_id = "map";
    map.header.stamp = rostime;
    if (octomap_msgs::binaryMapToMsg(global_map_->getOctree(), map)) {
        binary_map_pub_->publish(map);
    } else {
        RCLCPP_ERROR(get_logger(), "Error serializing OctoMap");
    }
}

}
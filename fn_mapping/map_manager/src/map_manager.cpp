// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "map_manager.h"


namespace finenav_2d {

MapManager::MapManager(const rclcpp::NodeOptions& options)
    : Node("map_manager", options),
        local_map_({20.0, 20.0, 20.0}, 0.5) // 初始化地图
    {
    RCLCPP_INFO(get_logger(), "MapManager initialized");

    std::chrono::duration<int> buffer_timeout(1);

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


}


void MapManager::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // 改写成message_filters实现的tf接收，文档......
    // 参考octomap_server

    //1. 读取点云的 frame_id
    std::string frame_id = msg->header.frame_id;
    RCLCPP_INFO(this->get_logger(), "Received pointcloud in frame: %s", frame_id.c_str());

    // 2. 将 PointCloud2 转成 Eigen::Vector3d 的点集合
    std::vector<Eigen::Vector3d> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        points.emplace_back(*iter_x, *iter_y, *iter_z);
    }


    for (const auto& p : points) {
        local_map_.atPosition(p) = 1;  //把对应栅格设置成占据
    }

    // moveTo()








    // 发布地图（可视化）
    publishLocalMap();


}

void MapManager::publishLocalMap() {

    // 获取local_map_中所有Occupied的格子
    std::vector<Index> Ocuppied_cells;
    local_map_.selectCellsByCondition(Ocuppied_cells, [](const uint8_t& value) {
        return value; // 假设 true 表示 Ocuppied
    });

    //创建Pointcloud2信息
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = this->now();
    cloud.height = 1;  // 非组织型点云
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
        pos = local_map_.getPosition(idx); // 获取对应的世界坐标

        *iter_x = pos.x(); ++iter_x;
        *iter_y = pos.y(); ++iter_y;
        *iter_z = pos.z(); ++iter_z;
    }

    // 4. 发布点云
    local_map_pub_->publish(cloud);
}




}

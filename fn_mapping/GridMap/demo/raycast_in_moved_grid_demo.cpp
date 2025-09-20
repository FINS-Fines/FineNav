// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>


#include "grid_map.hpp"
#include "cloud_publish_helper.hpp"

using namespace finenav_2d;

class MapManager : public rclcpp::Node {
public:
    explicit MapManager(const rclcpp::NodeOptions& options)
        : Node("map_manager", options), grid_map_({10.0, 10.0, 10.0}, 0.1) {

        // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapManager::timerCallback, this));

        grid_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map", rclcpp::QoS(1).transient_local());
        ray_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ray_points", rclcpp::QoS(1).transient_local());
        sensor_origin_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_origin", rclcpp::QoS(1).transient_local());

        auto points = std::vector<Position> { // 世界坐标系下，位置不变
                    {0.5, 0.5, 0},
                    {0, -1, -0.1}
        };

        pub_helper_.configure(grid_map_pub_, true, "map");
        for (auto & pt : points) {
            pub_helper_.addPoint(pt.x(), pt.y(), pt.z());
        }
        pub_helper_.publish(this->now());


        Position moved_distance = {1, 0, 0};
        Position sensor_origin{0, 0, 0.5};

        grid_map_.moveTo( grid_map_.getOrigin() + moved_distance );
        sensor_origin += moved_distance; // 世界坐标系下，机器人移动

        pub_helper_.configure(sensor_origin_pub_, true, "map");
        pub_helper_.addPoint(sensor_origin.x(), sensor_origin.y(), sensor_origin.z());
        pub_helper_.publish(this->now());

        // raycast
        pub_helper_.configure(ray_pub_, true, "map");
        for (auto & pt : points) {
            std::vector<Index> ray_indices;
            // if (grid_map_.rayCast(sensor_origin, pt, ray_indices)) {
                // 遍历光线上的栅格
                grid_map_.rayCast(sensor_origin, pt, ray_indices);
                for (size_t i = 0; i + 1 < ray_indices.size(); ++i) {
                    grid_map_.at(ray_indices[i]) = 0;

                    auto pos = grid_map_.getPosition(ray_indices[i]); // 记录ray穿过的idx
                    pub_helper_.addPoint(pos.x(), pos.y(), pos.z());
                }
            // }
        }
        pub_helper_.publish(this->now());

    }

private:

    GridMap<uint8_t> grid_map_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_pub_;  // 发布局部地图的点云消息
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ray_pub_;  // 发布局部地图的点云消息
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_origin_pub_;  // 发布局部地图的点云消息

    finenav_utils::CloudPublishHelper pub_helper_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapManager>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

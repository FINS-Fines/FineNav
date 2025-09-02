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

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapManager::timerCallback, this));

        grid_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map", 10 );
        ray_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ray_points", 10);
        sensor_origin_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_origin", 10);

    }

private:
    void timerCallback() {

        // 定义点云
        auto indices = std::vector<Index>{
            {5, 5, 0}, {0, -10, -1}, {20, 0, 0}, {50, 50, 0}
        };

        Index sensor_index{0, 0, 5};
        auto sensor_origin = grid_map_.getPosition(sensor_index);

        pub_helper_.configure(sensor_origin_pub_, true, "map");
        pub_helper_.addPoint(sensor_origin.x(), sensor_origin.y(), sensor_origin.z());
        pub_helper_.publish(this->now());

        // raycast
        pub_helper_.configure(ray_pub_, true, "map");
        for (auto & idx : indices) {
            std::vector<Index> ray_indices;
            if (grid_map_.rayCast(sensor_origin, grid_map_.getPosition(idx), ray_indices)) {
                // 遍历光线上的栅格
                for (size_t i = 0; i + 1 < ray_indices.size(); ++i) {
                    grid_map_.at(ray_indices[i]) = 0;

                    auto pos = grid_map_.getPosition(ray_indices[i]); // 记录ray穿过的idx
                    pub_helper_.addPoint(pos.x(), pos.y(), pos.z());
                }
            }
        }
        pub_helper_.publish(this->now());

        for (auto & idx : indices) {
            grid_map_.at(idx) = 255;
        }


        // 可视化GridMap
        pub_helper_.configure(grid_map_pub_, true, "map");
        for (auto it = grid_map_.begin(); it != grid_map_.end(); ++it) {
            Index idx = it.getIndex();
            Position pos = it.getPosition();
            if (*it == 255) {
                pub_helper_.addPoint(pos.x(), pos.y(), pos.z());
            }
        }
        pub_helper_.publish(this->now());
    }

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

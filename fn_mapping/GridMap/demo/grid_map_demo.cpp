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
        : Node("map_manager", options), grid_map_({5.0, 5.0, 5.0}, 0.1) {

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapManager::timerCallback, this));

        grid_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 10 );
        origin_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("map_origin_marker", 10);

        /******** 模拟初始化来了一帧点云，给原点为中心的2*2*2m范围赋值为点的高度 *******/
        for (int x = -20; x <= 20; ++x) {
            for (int y = -20; y <= 20; ++y) {
                for (int z = -20; z <= 20; ++z) {
                    grid_map_.atPosition({x * 0.05, y * 0.05, z * 0.05}) = z * 0.05;
                }
            }
        }

    }

private:
    void timerCallback() {
        /******** 模拟初始化来了一帧点云，给原点为中心的2*2*2m范围赋值为点的高度 *******/
        for (int x = -2; x <= 2; ++x) {
            for (int y = -2; y <= 2; ++y) {
                for (int z = -2; z <= 2; ++z) {
                    grid_map_.at({x, y, z}) = z * 0.05;
                }
            }
        }

        pub_helper_.configure(grid_map_pub_, true, "base_link");

        // 模拟机器人发生了移动，从原点移动了(0.1, 0.1，0.1)
        auto new_pos = Position{0.1, 0.1, 0.1} + grid_map_.getOrigin();
        grid_map_.moveTo(new_pos);

        // 用迭代器遍历栅格地图
        for (auto it = grid_map_.begin(); it != grid_map_.end(); ++it) {
            Index idx = it.getIndex();
            Position pos = it.getPosition();
            if (!std::isnan(*it)) {
                pub_helper_.addPoint(pos.x(), pos.y(), *it);
            }
        }

        pub_helper_.publish(this->now());
        publishOriginMarker();
    }

    /**
    * @brief 发布local_map_的origin作为可视化球
    */
    void publishOriginMarker() {
        // 创建可视化标记消息
        visualization_msgs::msg::Marker marker;

        // 设置标记的基本属性
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "map_origin";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 获取local_map_的origin位置
        Position origin = grid_map_.getOrigin();

        // 设置标记的位置
        marker.pose.position.x = origin.x();
        marker.pose.position.y = origin.y();
        marker.pose.position.z = origin.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // 设置标记的尺寸 (球的直径)
        marker.scale.x = 0.2;  // x方向直径
        marker.scale.y = 0.2;  // y方向直径
        marker.scale.z = 0.2;  // z方向直径

        // 设置标记的颜色 (红色球)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;  // 透明度

        // 设置标记的生存时间
        marker.lifetime = rclcpp::Duration::from_seconds(0);  // 0表示永久存在

        // 发布标记
        origin_pub_->publish(marker);
    }

    GridMap<float> grid_map_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_pub_;  // 发布局部地图的点云消息
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr origin_pub_;  // 发布原点可视化的消息

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

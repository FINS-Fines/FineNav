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

        timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&MapManager::timerCallback, this));

        grid_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map", 10 );
        origin_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_origin", 10);

        insertPointCloud();
    }

private:
    void insertPointCloud() {
        // 模拟世界坐标系中 x=0 位置有一面固定的墙，假设点云已经转换到世界坐标系下
        for (int y = -20; y <= 20; ++y) {
            for (int z = -20; z <= 20; ++z) {
                Position pos = {0.0, y*0.1, z*0.1};
                grid_map_.atPosition(pos) = 255;
            }
        }
    }

    void timerCallback() {
        static Position accum_pos{0, 0, 0};
        Position step_pos{0.06, 0.0, 0.0};

        accum_pos += step_pos;
        grid_map_.moveTo(accum_pos);

        // 模拟新的点云输入，
        // insertPointCloud();

        // 用迭代器遍历栅格地图
        pub_helper_.configure(grid_map_pub_, true, "map");
        for (auto it = grid_map_.begin(); it != grid_map_.end(); ++it) {
            Index idx = it.getIndex();
            Position pos = it.getPosition();
            if (*it == 255) {
                pub_helper_.addPoint(pos.x(), pos.y(), pos.z());
            }
        }
        pub_helper_.publish(this->now());

        // 发布地图原点
        pub_helper_.configure(origin_pub_, true, "map");
        auto origin = grid_map_.getOrigin();
        pub_helper_.addPoint(origin.x(), origin.y(), origin.z());
        pub_helper_.publish(this->now());
    }


    GridMap<uint8_t> grid_map_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_pub_;  // 发布局部地图的点云消息
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr origin_pub_;  // 发布局部地图的点云消息

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

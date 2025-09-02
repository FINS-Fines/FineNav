// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "octomap_server.hpp"
#include "cloud_publish_helper.hpp"

using namespace finenav_2d;

class OctoMapServerDemo : public rclcpp::Node {
public:
    explicit OctoMapServerDemo(const rclcpp::NodeOptions& options) : Node("octomap_server_demo", options) {

        min_pt_ = octomap::point3d(-1.0, -1.0, -1.0);
        max_pt_ = octomap::point3d(1.0, 1.0, 1.0);
        move_step_ = octomap::point3d(0.1, 0.1, 0.1);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(3000), std::bind(&OctoMapServerDemo::timerCallback, this));

        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("octomap", 10 );
        test_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("test_cloud", 10 );

        // 将(-1,-1,-1)到(1,1,1)范围内的点设置为占据
        for(int x=-10; x<10; x++)
        {
            for(int y=-10; y<10; y++)
            {
                for(int z=-10; z<10; z++)
                {
                    octomap::point3d endpoint((float) x*0.1f, (float) y*0.1f, (float) z*0.1f); // 计算占据点的坐标
                    octomap_server_.getOctree().updateNode(endpoint, true); // true 表示将该node标记为占据
                }
            }
        }

    }

private:
    void timerCallback() {
        static octomap::point3d moved_distance(0,0,0);

        // 移动地图
        min_pt_ += moved_distance;
        max_pt_ += moved_distance;

        // 可视化removed区域
        pub_helper_.configure(test_pub, true, "map");
        auto callback = [this](octomap::OcTree::leaf_bbx_iterator& it) {
            if (octomap_server_.getOctree().isNodeOccupied(*it)) { // 如果为占据则发布
                pub_helper_.addPoint(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
            }
        };
        octomap_server_.traverseMoveDifferenceRegion(min_pt_, max_pt_, move_step_, callback);
        pub_helper_.publish(this->now());


        // 发布地图
        pub_helper_.configure(map_pub_, true, "map");
        auto tree = octomap_server_.getOctree();
        for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it)
        {
            if (tree.isNodeOccupied(*it)) { // 如果为占据则发布
                pub_helper_.addPoint(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
            }

            //manipulate node, e.g.:
            // std::cout << "Node center: " << it.getCoordinate() << std::endl; // 叶子节点的中心坐标
            // std::cout << "Node size: " << it.getSize() << std::endl; // 叶子节点的Size
            // std::cout << "Node value: " << it->getValue() << std::endl; // 返回的是节点的 log-odds 概率值
        }
        pub_helper_.publish(this->now());

        moved_distance += move_step_; // 累计移动距离
    }

    OctoMapServer octomap_server_;  // 八叉树分辨率为0.1米

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pub;  // 发布局部地图的点云消息

    finenav_utils::CloudPublishHelper pub_helper_;
    rclcpp::TimerBase::SharedPtr timer_;

    octomap::point3d min_pt_;
    octomap::point3d max_pt_;
    octomap::point3d move_step_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctoMapServerDemo>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

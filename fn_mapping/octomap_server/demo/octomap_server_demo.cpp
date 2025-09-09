// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/HeightOcTree.h>

#include "octomap_server.hpp"
#include "cloud_publish_helper.hpp"

using namespace finenav_2d;

class OctoMapServerDemo : public rclcpp::Node {
public:
    explicit OctoMapServerDemo(const rclcpp::NodeOptions& options)
        : Node("octomap_server_demo", options), octomap_server_(0.1) {

        min_pt_ = octomap::point3d(-1.0, -1.0, -1.0);
        max_pt_ = octomap::point3d(1.0, 1.0, 1.0);
        move_step_ = octomap::point3d(0.1, 0.1, 0.1);
        expand_to_max_depth_ = true; // 展开到最大深度;

        timer_ = this->create_wall_timer(std::chrono::milliseconds(3000), std::bind(&OctoMapServerDemo::timerCallback, this));

        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("octomap", 10 );
        test_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("test_cloud", 10 );

        // 将(-1,-1,-1)到(1,1,1)范围内的点设置为占据
        for(int x=-20; x<20; x++)
        {
            for(int y=-20; y<20; y++)
            {
                for(int z=-20; z<20; z++)
                {
                    octomap::point3d endpoint( // 0.05小于八叉树分辨率，防止浮点数运算精度问题造成更新八叉树错误
                        static_cast<float>(x)*0.05f,
                        static_cast<float>(y)*0.05f,
                        static_cast<float>(z)*0.05f
                        );
                    octomap_server_.getOctree().updateNodeWithHeight(endpoint, endpoint.z()); // 此时已经设置为占据
                }
            }
        }
        octomap_server_.getOctree().prune(); // 剪枝，删除那些8个子节点都是相同状态的父节点
    }

private:
    void timerCallback() {
        // 可视化removed区域
        pub_helper_.configure(test_pub, true, "map");
        OctoMapServer::TraverseCallback callback;
        callback = [this](OctoMapServer::IteratorBase* it) {
            if (octomap_server_.getOctree().isNodeOccupied(**it)) { // 先解引用迭代器指针，再解引用迭代器得到节点
                pub_helper_.addPoint(it->getCoordinate().x(), it->getCoordinate().y(), it->getCoordinate().z());
            }
            auto max_depth = octomap_server_.getOctree().getTreeDepth();
            if ((*it)->isHeightSet() && it->getDepth() == max_depth) {
                float height = (*it)->getHeight();
                // Do something
            }
        };

        octomap_server_.traverseMoveDifferenceRegion(min_pt_, max_pt_, move_step_, callback, expand_to_max_depth_);
        pub_helper_.publish(this->now());

        // 发布地图
        pub_helper_.configure(map_pub_, true, "map");
        const auto& tree = octomap_server_.getOctree();
        for(octomap::HeightOcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it)
        {
            if (tree.isNodeOccupied(*it)) { // 如果为占据则发布
                // 这里也可以获取高度值
                if (it->isHeightSet() && it.getDepth() == tree.getTreeDepth()) {
                    pub_helper_.addPoint(it.getCoordinate().x(), it.getCoordinate().y(), it->getHeight());
                }

            }
        }
        pub_helper_.publish(this->now());

        // 地图移动
        min_pt_ += move_step_;
        max_pt_ += move_step_;
    }

    OctoMapServer octomap_server_;  // 八叉树分辨率为0.1米

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pub;  // 发布局部地图的点云消息

    finenav_utils::CloudPublishHelper pub_helper_;
    rclcpp::TimerBase::SharedPtr timer_;

    octomap::point3d min_pt_;
    octomap::point3d max_pt_;
    octomap::point3d move_step_;

    bool expand_to_max_depth_; // 是否展开到最大深度

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctoMapServerDemo>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <queue>

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
                    octomap_server_.getOctree().updateNode(endpoint, true);
                }
            }
        }
        // octomap_server_.getOctree().prune(); // 剪枝，删除那些8个子节点都是相同状态的父节点
    }

private:
    void timerCallback() {
        // 可视化removed区域
        pub_helper_.configure(test_pub, true, "map");
        auto callback = [this](octomap::OcTree::leaf_bbx_iterator& it) {
            this->processNode(it);
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
        }
        pub_helper_.publish(this->now());

        // 地图移动
        min_pt_ += move_step_;
        max_pt_ += move_step_;
    }

    void processNode(octomap::OcTree::leaf_bbx_iterator& it) {
        if (!expand_to_max_depth_) {
            if (octomap_server_.getOctree().isNodeOccupied(*it)) { // 如果为占据则发布
                pub_helper_.addPoint(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
            }
            return;
        }

        // 展开到最大深度
        struct NodeToExpand {
            octomap::point3d center;
            double size;
            unsigned int depth;
        };

        const unsigned int max_depth = octomap_server_.getOctree().getTreeDepth();
        std::queue<NodeToExpand> nodes_to_process; // 节点处理队列

        // 初始节点入队
        nodes_to_process.emplace(it.getCoordinate(), it.getSize(), it.getDepth());

        while (!nodes_to_process.empty()) {
            NodeToExpand current = nodes_to_process.front();
            nodes_to_process.pop();

            // 如果到达最大深度，发布节点
            if (current.depth >= max_depth) {
                if (isInMoveDifferenceRegion(current.center)) { // 检测该center是否在L形区域内
                    pub_helper_.addPoint(current.center.x(), current.center.y(), current.center.z());
                }
                continue;
            }

            // 计算8个子节点
            double half_size = current.size / 2.0;
            double quarter_size = half_size / 2.0;

            static const std::vector<std::array<int, 3>> offsets = {
                {{-1, -1, -1}}, {{1, -1, -1}}, {{-1, 1, -1}}, {{1, 1, -1}},
                {{-1, -1, 1}}, {{1, -1, 1}}, {{-1, 1, 1}}, {{1, 1, 1}}
            };

            for (const auto& offset : offsets) {
                octomap::point3d child_center(
                    current.center.x() + offset[0] * quarter_size,
                    current.center.y() + offset[1] * quarter_size,
                    current.center.z() + offset[2] * quarter_size
                );
                nodes_to_process.emplace(child_center, half_size, current.depth + 1);
            }
        }
    }

    bool isInMoveDifferenceRegion(const octomap::point3d& point) {
        // 原始包围盒
        octomap::point3d original_min = min_pt_;
        octomap::point3d original_max = max_pt_;

        // 移动后的包围盒
        octomap::point3d moved_min = original_min + move_step_;
        octomap::point3d moved_max = original_max + move_step_;

        // 检查点是否在原始包围盒内但不在移动后包围盒内（即removed区域）
        bool in_original = (point.x() >= original_min.x() && point.x() <= original_max.x()) &&
                          (point.y() >= original_min.y() && point.y() <= original_max.y()) &&
                          (point.z() >= original_min.z() && point.z() <= original_max.z());

        bool in_moved = (point.x() >= moved_min.x() && point.x() <= moved_max.x()) &&
                       (point.y() >= moved_min.y() && point.y() <= moved_max.y()) &&
                       (point.z() >= moved_min.z() && point.z() <= moved_max.z());

        return in_original && !in_moved;  // L形区域 = 原始区域 - 移动后区域
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

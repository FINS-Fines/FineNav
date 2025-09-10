#include "pct_planner.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using FollowPath = nav2_msgs::action::FollowPath;
using ComputePathServer = rclcpp_action::Server<ComputePathToPose>;
using FollowPathClient = rclcpp_action::Client<FollowPath>;

PctPlanner::PctPlanner(const rclcpp::NodeOptions& options) : Node("pct_planner", options) {
    /********* Parameters for PctPlanner *********/
    this->declare_parameter("pcd_file_path", "");
    this->declare_parameter("tomography_visualize_", false);
    pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();
    tomography_visualize_ = this->get_parameter("tomography_visualize_").as_bool();

    // TODO: FOR DEBUG
    pcd_file_path_ = "/home/fins/Desktop/nav_ws/FineNav2D/fn_fine_pct/rsc/pcd/fine.pcd";
    tomography_visualize_ = true;

    /********* Parameters for Tomography *********/
    this->declare_parameter("resolution", tomography_config.resolution);
    this->declare_parameter("slice_dh", tomography_config.slice_dh);
    this->declare_parameter("ground_h", tomography_config.ground_h);
    this->declare_parameter("interval_min", tomography_config.interval_min);
    this->declare_parameter("slope_max", tomography_config.slope_max);
    this->declare_parameter("slope_cost_ratio", tomography_config.slope_cost_ratio);
    this->declare_parameter("cost_barrier", tomography_config.cost_barrier);
    this->declare_parameter("safe_margin", tomography_config.safe_margin);
    this->declare_parameter("inflation", tomography_config.inflation);

    tomography_config.resolution = this->get_parameter("resolution").as_double();
    tomography_config.slice_dh = this->get_parameter("slice_dh").as_double();
    tomography_config.ground_h = this->get_parameter("ground_h").as_double();
    tomography_config.interval_min = this->get_parameter("interval_min").as_double();
    tomography_config.slope_max = this->get_parameter("slope_max").as_double();
    tomography_config.slope_cost_ratio = this->get_parameter("slope_cost_ratio").as_double();
    tomography_config.cost_barrier = this->get_parameter("cost_barrier").as_double();
    tomography_config.safe_margin = this->get_parameter("safe_margin").as_double();
    tomography_config.inflation = this->get_parameter("inflation").as_double();

    tomography_ = std::make_unique<Tomography>(tomography_config);
    path_finder_ = std::make_unique<Astar>();

    initPlanner();

    // 初始化Action服务器和客户端
    compute_path_server_ = rclcpp_action::create_server<ComputePathToPose>(
        this, "compute_path_to_pose",
        std::bind(&PctPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PctPlanner::handle_cancel, this, std::placeholders::_1),
        std::bind(&PctPlanner::handle_accepted, this, std::placeholders::_1));

    // 创造一个虚假的compute_path_through_poses服务器, 实际没有作用
    // 模板类型改为 nav2_msgs::action::ComputePathThroughPoses
    compute_path_server_2 = rclcpp_action::create_server<nav2_msgs::action::ComputePathThroughPoses>(
        this, "compute_path_through_poses",
        // 回调函数中的类型也要同步修改
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const nav2_msgs::action::ComputePathThroughPoses::Goal>) {
            return rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathThroughPoses>>) {
            return rclcpp_action::CancelResponse::REJECT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathThroughPoses>>
                   goal_handle) {
            // 这里可以添加实际处理逻辑（如果需要）
        });
    // follow_path_client_ = rclcpp_action::create_client<FollowPath>(
    //     this, "follow_path");

    // 订阅里程计获取当前位置
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
            std::lock_guard<std::mutex> lock(position_mutex_);
            robot_current_position_[0] = odom_msg->pose.pose.position.x;
            robot_current_position_[1] = odom_msg->pose.pose.position.y;
            robot_current_position_[2] = odom_msg->pose.pose.position.z;

            RCLCPP_DEBUG(this->get_logger(), "Robot current position: x=%.2f, y=%.2f, z=%.2f",
                         robot_current_position_[0], robot_current_position_[1], robot_current_position_[2]);
        });

    if (path_visualize_) {
        auto qos2 = rclcpp::QoS(1).transient_local();
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", qos2);
    }

    // 可视化tomography
    if (tomography_visualize_) {
        auto qos = rclcpp::QoS(1).transient_local();
        tomography_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tomography_layers", qos);
        publishTomography();
    }

    RCLCPP_INFO(this->get_logger(), "PCT Planner initialized successfully");
}

rclcpp_action::GoalResponse PctPlanner::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                    std::shared_ptr<const ComputePathToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received path planning request");
    (void)uuid;
    // 可以在这里添加对目标的验证逻辑
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PctPlanner::handle_cancel(const std::shared_ptr<ComputePathGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel path planning");
    (void)goal_handle;
    // 如果可能的话，在这里添加停止规划的逻辑
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PctPlanner::handle_accepted(const std::shared_ptr<ComputePathGoalHandle> goal_handle) {
    // 使用异步方式处理请求，避免阻塞主线程
    std::thread{std::bind(&PctPlanner::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void PctPlanner::execute(const std::shared_ptr<ComputePathGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing path planning");
    const auto goal = goal_handle->get_goal();

    // 获取机器人当前位置作为起点
    Eigen::Vector3d start_real;
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        // 使用实际消息结构中的use_start字段
        if (goal->use_start) {
            start_real[0] = goal->start.pose.position.x;
            start_real[1] = goal->start.pose.position.y;
            start_real[2] = goal->start.pose.position.z;
        } else {
            start_real = robot_current_position_;
        }
    }

    // 创建实际系下的起点和终点
    Eigen::Vector3d goal_real(goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal.pose.position.z);

    // 转换到地图系
    Eigen::Vector3i start_map, goal_map;
    start_map[0] = static_cast<int>(start_real[2] / tomography_config.slice_dh);  // layer
    start_map[1] =
        static_cast<int>(std::round((start_real[1] - tomography_->getCenter()[1]) / tomography_config.resolution)) +
        tomography_->getMapDimY() / 2;  // row
    start_map[2] =
        static_cast<int>(std::round((start_real[0] - tomography_->getCenter()[0]) / tomography_config.resolution)) +
        tomography_->getMapDimX() / 2;  // col

    goal_map[0] = static_cast<int>(goal_real[2] / tomography_config.slice_dh);  // layer
    goal_map[1] =
        static_cast<int>(std::round((goal_real[1] - tomography_->getCenter()[1]) / tomography_config.resolution)) +
        tomography_->getMapDimY() / 2;  // row
    goal_map[2] =
        static_cast<int>(std::round((goal_real[0] - tomography_->getCenter()[0]) / tomography_config.resolution)) +
        tomography_->getMapDimX() / 2;  // col

    RCLCPP_INFO(this->get_logger(), "Start map idx: layer %d, row %d, col %d", start_map[0], start_map[1],
                start_map[2]);
    RCLCPP_INFO(this->get_logger(), "Goal map idx: layer %d, row %d, col %d", goal_map[0], goal_map[1], goal_map[2]);

    // 执行路径搜索
    bool path_found = path_finder_->search(start_map, goal_map);
    if (!path_found) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find a path from start to goal");
        auto result = std::make_shared<ComputePathToPose::Result>();
        goal_handle->abort(result);
        return;
    }

    // 获取路径并转换为ROS消息格式
    auto path_indices = path_finder_->getPath();
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    for (const auto& idx : path_indices) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        // 从地图系转换到实际系
        pose.pose.position.x =
            (static_cast<double>(idx[2]) - tomography_->getMapDimX() / 2) * tomography_config.resolution +
            tomography_->getCenter()[0];
        pose.pose.position.y =
            (static_cast<double>(idx[1]) - tomography_->getMapDimY() / 2) * tomography_config.resolution +
            tomography_->getCenter()[1];
        pose.pose.position.z = static_cast<double>(idx[0]) / 100;
        pose.pose.orientation.w = 1.0;  // 无旋转
        path_msg.poses.push_back(pose);

        // 发布路径用于调试
        if (path_visualize_) {
            path_pub_->publish(path_msg);
        }
    }

    // 返回规划结果
    auto result = std::make_shared<ComputePathToPose::Result>();
    result->path = path_msg;
    // 设置规划时间（根据实际消息结构添加）
    result->planning_time = rclcpp::Duration::from_nanoseconds((this->now() - path_msg.header.stamp).nanoseconds());
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Path planning completed successfully");
}

void PctPlanner::initPlanner() const {
    // 加载PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud) == -1) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load PCD file: " << pcd_file_path_);
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Loaded PCD file: " << pcd_file_path_ << " with " << cloud->size() << " points");

    // voxel滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(tomography_config.resolution / 2, tomography_config.resolution / 2,
                             tomography_config.resolution / 2);
    voxel_filter.filter(*cloud);
    RCLCPP_INFO_STREAM(this->get_logger(), "Filtered cloud has " << cloud->size() << " points");

    // 执行tomography流程
    tomography_->setInputCloud(cloud);
    tomography_->startAlgorithm();

    // 创建Astar
    auto layers = tomography_->getOutputLayers();
    int a_star_cost_threshold = 20;  // TODO: 参数
    double step_cost_weight = 0.2;

    path_finder_->initialize(a_star_cost_threshold, step_cost_weight, layers, HeuristicType::kDiagonal);
}

void PctPlanner::publishTomography() const {
    auto layers = tomography_->getOutputLayers();
    auto layers_num = layers.trav_cost.layers.size();

    // 创建带颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->reserve(tomography_->getMapDimX() * tomography_->getMapDimY() * layers_num);

    // 按照cost着色点云，高cost红色，低cost蓝色
    for (size_t k = 0; k < layers_num; ++k) {
        for (int x = 0; x < tomography_->getMapDimX(); ++x) {
            for (int y = 0; y < tomography_->getMapDimY(); ++y) {
                if (std::isnan(layers.trav_cost(x, y, k))) {
                    continue;
                }

                pcl::PointXYZRGB point;
                // 坐标转换
                point.x =
                    tomography_->getCenter()[0] + (x - tomography_->getMapDimX() / 2) * tomography_->getResolution();
                point.y =
                    tomography_->getCenter()[1] + (y - tomography_->getMapDimY() / 2) * tomography_->getResolution();
                point.z = layers.ground(x, y, k);

                // cost to color
                float cost = layers.trav_cost(x, y, k);
                if (cost > tomography_config.cost_barrier) {
                    cost = tomography_config.cost_barrier;
                }
                float ratio = cost / tomography_config.cost_barrier;

                // 高cost红色，低cost蓝色
                uint8_t R = static_cast<uint8_t>(ratio * 255);
                uint8_t G = static_cast<uint8_t>(0);
                uint8_t B = static_cast<uint8_t>((1 - ratio) * 255);

                point.r = R;
                point.g = G;
                point.b = B;

                colored_cloud->push_back(point);
            }
        }
    }

    // 转换并发布点云
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*colored_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = this->now();

    // 保存结果
    const std::string pcd_path = "tomography_results.pcd";
    if (pcl::io::savePCDFileBinary(pcd_path, *colored_cloud) == 0) {
        RCLCPP_INFO(this->get_logger(), "Successfully saved to %s", pcd_path.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file!");
    }

    tomography_pub_->publish(cloud_msg);
}

// void PctPlanner::sendPathToFollow(const nav_msgs::msg::Path& path) {
//     RCLCPP_INFO(this->get_logger(), "Sending path to FollowPath action server");

//     // 创建FollowPath请求
//     auto goal_msg = FollowPath::Goal();
//     goal_msg.path = path;
//     goal_msg.controller_id = "";  // 使用默认控制器

//     // 等待FollowPath服务器就绪
//     if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
//         RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available");
//         return;
//     }

//     // 发送路径给控制器
//     auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
//     send_goal_options.goal_response_callback =
//         [this](const rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr & goal_handle) {
//             if (!goal_handle) {
//                 RCLCPP_ERROR(this->get_logger(), "FollowPath request rejected");
//             } else {
//                 RCLCPP_INFO(this->get_logger(), "FollowPath request accepted");
//             }
//         };

//     send_goal_options.result_callback =
//         [this](const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult & result) {
//             switch (result.code) {
//                 case rclcpp_action::ResultCode::SUCCEEDED:
//                     RCLCPP_INFO(this->get_logger(), "Path following completed successfully");
//                     break;
//                 case rclcpp_action::ResultCode::ABORTED:
//                     RCLCPP_ERROR(this->get_logger(), "Path following aborted");
//                     return;
//                 case rclcpp_action::ResultCode::CANCELED:
//                     RCLCPP_INFO(this->get_logger(), "Path following canceled");
//                     return;
//                 default:
//                     RCLCPP_ERROR(this->get_logger(), "Path following failed with unknown code");
//                     return;
//             }
//         };

//     follow_path_client_->async_send_goal(goal_msg, send_goal_options);
// }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<PctPlanner>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
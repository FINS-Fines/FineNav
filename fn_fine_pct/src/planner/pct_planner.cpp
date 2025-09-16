#include "pct_planner.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using FollowPath = nav2_msgs::action::FollowPath;
using ComputePathServer = rclcpp_action::Server<ComputePathToPose>;
using FollowPathClient = rclcpp_action::Client<FollowPath>;

const double tolerance = 1e-6;                                                // 收敛容差
const int max_its = 3000;                                                      // 最大迭代次数
const double w_data = 0.2;                                                     // 数据权重（保留原始路径）
const double w_smooth = 0.3;                                                   // 平滑权重（控制平滑程度）
const bool do_refinement = true;                                               // 是否二次细化
const int refinement_num = 100;                                                  // 细化次数
const bool enforce_path_inversion = false;                                      // 是否按方向分割路径段
const rclcpp::Duration max_smooth_time = rclcpp::Duration::from_seconds(15);  // 最大平滑时间（可调整）

PctPlanner::PctPlanner(const rclcpp::NodeOptions& options) : Node("pct_planner", options) {
    /********* Parameters for PctPlanner *********/
    this->declare_parameter("pcd_file_path_", "");
    this->declare_parameter("tomography_visualize_", false);
    pcd_file_path_ = this->get_parameter("pcd_file_path_").as_string();
    tomography_visualize_ = this->get_parameter("tomography_visualize_").as_bool();

    // TODO: FOR DEBUG
    pcd_file_path_ = "/home/fins/Desktop/nav_ws/FineNav2D/fn_fine_pct/rsc/pcd/final_map_v16.pcd";
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
    this->declare_parameter("kernal_size", tomography_config.kernal_size);

    tomography_config.resolution = this->get_parameter("resolution").as_double();
    tomography_config.slice_dh = this->get_parameter("slice_dh").as_double();
    tomography_config.ground_h = this->get_parameter("ground_h").as_double();
    tomography_config.interval_min = this->get_parameter("interval_min").as_double();
    tomography_config.slope_max = this->get_parameter("slope_max").as_double();
    tomography_config.slope_cost_ratio = this->get_parameter("slope_cost_ratio").as_double();
    tomography_config.cost_barrier = this->get_parameter("cost_barrier").as_double();
    tomography_config.safe_margin = this->get_parameter("safe_margin").as_double();
    tomography_config.inflation = this->get_parameter("inflation").as_double();
    tomography_config.kernal_size = this->get_parameter("kernal_size").as_int();

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
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const nav2_msgs::action::ComputePathThroughPoses::Goal>) {
            return rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathThroughPoses>>) {
            return rclcpp_action::CancelResponse::REJECT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathThroughPoses>>
                   goal_handle) {});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
        // 使用实际消息结构中的use_start字段
        if (goal->use_start) {
            start_real[0] = goal->start.pose.position.x;
            start_real[1] = goal->start.pose.position.y;
            start_real[2] = goal->start.pose.position.z;
        } else {
            bool query_success = get_robot_position_manual(start_real);
            if (!query_success) {
                // 查询失败时的处理（例如使用上一次缓存的位置或中止规划）
                RCLCPP_ERROR(this->get_logger(), "无法获取机器人当前位置，规划中止");
                auto result = std::make_shared<ComputePathToPose::Result>();
                goal_handle->abort(result);
                return;
            }
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
        ;
        pose.pose.orientation.w = 1.0;  // 无旋转
        path_msg.poses.push_back(pose);

        // 发布路径用于调试
    }

    // TODO: 平滑路径
    // -------------------------- 2. 核心平滑逻辑 --------------------------
    std::vector<PathSegment> path_segments{{0u, static_cast<unsigned int>(path_msg.poses.size() - 1)}};
    if (enforce_path_inversion) {
        path_segments = findDirectionalPathSegments(path_msg);
    }

    const auto start_time = std::chrono::steady_clock::now();
    double time_remaining = max_smooth_time.seconds();
    int refinement_ctr = 0;
    bool reversing_segment;
    nav_msgs::msg::Path curr_path_segment;
    curr_path_segment.header = path_msg.header;

    for (auto& segment : path_segments) {
        if (segment.end - segment.start <= 3)
            continue;

        curr_path_segment.poses.clear();
        std::copy(path_msg.poses.begin() + segment.start, path_msg.poses.begin() + segment.end + 1,
                  std::back_inserter(curr_path_segment.poses));

        const auto now = std::chrono::steady_clock::now();
        time_remaining = max_smooth_time.seconds() -
                         std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time).count();
        if (time_remaining <= 0) {
            RCLCPP_WARN(this->get_logger(), "Smoothing time exhausted, skip remaining segments");
            break;
        }

        // 段内平滑实现
        const auto seg_start_time = std::chrono::steady_clock::now();
        const rclcpp::Duration seg_max_dur = rclcpp::Duration::from_seconds(time_remaining);
        int its = 0;
        double change = tolerance;
        const unsigned int path_size = curr_path_segment.poses.size();
        nav_msgs::msg::Path new_path = curr_path_segment;
        nav_msgs::msg::Path last_path = curr_path_segment;

        while (change >= tolerance) {
            its++;
            change = 0.0;

            if (its >= max_its) {
                RCLCPP_WARN(this->get_logger(), "Smoothing iterations exceed limit (%d), use last valid path", max_its);
                curr_path_segment = last_path;
                updateApproximatePathOrientations(curr_path_segment, reversing_segment);
                break;
            }

            const auto seg_now = std::chrono::steady_clock::now();
            const rclcpp::Duration seg_timespan = rclcpp::Duration::from_seconds(
                std::chrono::duration_cast<std::chrono::duration<double>>(seg_now - seg_start_time).count());
            if (seg_timespan > seg_max_dur) {
                RCLCPP_WARN(this->get_logger(), "Smoothing time exceed limit (%.2fs), use last valid path",
                            time_remaining);
                curr_path_segment = last_path;
                updateApproximatePathOrientations(curr_path_segment, reversing_segment);
                throw std::runtime_error("Smoothing timed out");
            }

            for (unsigned int i = 1; i < path_size - 1; ++i) {
                for (unsigned int dim = 0; dim < 2; ++dim) {
                    const double x_i = getFieldByDim(curr_path_segment.poses[i], dim);
                    double y_i = getFieldByDim(new_path.poses[i], dim);
                    const double y_m1 = getFieldByDim(new_path.poses[i - 1], dim);
                    const double y_ip1 = getFieldByDim(new_path.poses[i + 1], dim);
                    const double y_i_org = y_i;

                    y_i += w_data * (x_i - y_i) + w_smooth * (y_ip1 + y_m1 - 2.0 * y_i);
                    setFieldByDim(new_path.poses[i], dim, y_i);
                    change += std::abs(y_i - y_i_org);
                }
            }

            last_path = new_path;
        }

        // 二次细化
        if (do_refinement && refinement_ctr < refinement_num) {
            refinement_ctr++;
            const auto refine_now = std::chrono::steady_clock::now();
            const double refine_remaining =
                time_remaining -
                std::chrono::duration_cast<std::chrono::duration<double>>(refine_now - start_time).count();
            if (refine_remaining > 0) {
                nav_msgs::msg::Path refined_path = curr_path_segment;
                int refine_its = 0;
                double refine_change = tolerance;
                nav_msgs::msg::Path refine_last = refined_path;
                while (refine_change >= tolerance && refine_its < max_its / 2) {
                    refine_its++;
                    refine_change = 0.0;
                    nav_msgs::msg::Path refine_new = refined_path;
                    for (unsigned int i = 1; i < refined_path.poses.size() - 1; ++i) {
                        for (unsigned int dim = 0; dim < 2; ++dim) {
                            const double x_i = getFieldByDim(curr_path_segment.poses[i], dim);
                            double y_i = getFieldByDim(refine_new.poses[i], dim);
                            const double y_m1 = getFieldByDim(refine_new.poses[i - 1], dim);
                            const double y_ip1 = getFieldByDim(refine_new.poses[i + 1], dim);
                            const double y_i_org = y_i;

                            y_i += w_data * 0.5 * (x_i - y_i) + w_smooth * 1.2 * (y_ip1 + y_m1 - 2.0 * y_i);
                            setFieldByDim(refine_new.poses[i], dim, y_i);
                            refine_change += std::abs(y_i - y_i_org);
                        }
                    }
                    refine_last = refine_new;
                    refined_path = refine_new;
                }
                curr_path_segment = refined_path;
            }
        }

        updateApproximatePathOrientations(curr_path_segment, reversing_segment);
        std::copy(curr_path_segment.poses.begin(), curr_path_segment.poses.end(),
                  path_msg.poses.begin() + segment.start);
    }

    if (path_visualize_) {
        path_pub_->publish(path_msg);
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
    // 从八叉树文件加载点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // OctoMapServer octomap(0.02);
    // octomap.openFile(octomap_file_path_);
    // const auto& tree = octomap.getOctree();
    // for (octomap::HeightOcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it) {

    //     auto max_depth = octomap.getOctree().getTreeDepth();
    //     if (it->isHeightSet() && it.getDepth() == tree.getTreeDepth()) {
    //     // if (tree.isNodeOccupied(*it)) {  // 如果为占据则发布
    //         pcl::PointXYZ point;
    //         point.x = it.getX();  // 获取x坐标
    //         point.y = it.getY();  // 获取y坐标
    //         point.z = it->getHeight();
    //         cloud->push_back(point);

    //         if(std::isnan(it->getHeight())) {
    //             RCLCPP_WARN(this->get_logger(), "!!!!!! Height is NaN at (%f, %f)", it.getX(), it.getY());
    //         }
    //     }
    // }
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "Loaded OctoMap file: " << octomap_file_path_ << " with " << cloud->size() << " points");

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

// void PctPlanner::update_robot_position() {
//     try {
//         // 查询"base_link"相对于"map"的最新变换
//         std::string target_frame = "base_link";  // 子坐标系（机器人基坐标系）
//         std::string source_frame = "map";        // 父坐标系（地图坐标系）
//         rclcpp::Time time = rclcpp::Time(0);     // 0表示获取最新的变换

//         // 阻塞查询，超时时间500ms（避免永久阻塞）
//         geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
//             source_frame,
//             target_frame,
//             time,
//             rclcpp::Duration::from_seconds(0.5)
//         );

//         // 提取位置信息并加锁保护
//         std::lock_guard<std::mutex> lock(position_mutex_);
//         robot_current_position_[0] = transform.transform.translation.x;  // x坐标
//         robot_current_position_[1] = transform.transform.translation.y;  // y坐标
//         robot_current_position_[2] = transform.transform.translation.z;  // z坐标（2D场景通常为0）

//         // 打印DEBUG日志（可选）
//         RCLCPP_DEBUG(this->get_logger(),
//             "Updated robot position: x=%.2f, y=%.2f, z=%.2f",
//             robot_current_position_[0],
//             robot_current_position_[1],
//             robot_current_position_[2]
//         );

//     } catch (tf2::TransformException& ex) {
//         // 处理查询失败（如TF未发布、超时等）
//         RCLCPP_WARN(this->get_logger(), "Failed to get TF (map -> base_link): %s", ex.what());
//         // 失败时可不更新位置，保持上一次有效数据
//     }
// }
// 在pct_planner.cpp中实现
bool PctPlanner::get_robot_position_manual(Eigen::Vector3d& position) {
    try {
        // 查询"map"到"base_link"的最新变换
        std::string source_frame = "map";
        std::string target_frame = "base_link";
        rclcpp::Time time = rclcpp::Time(0);  // 获取最新变换

        // 阻塞查询，超时时间1秒（可根据需求调整）
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(source_frame, target_frame, time, rclcpp::Duration::from_seconds(1.0));

        // 提取位置信息
        position[0] = transform.transform.translation.x;
        position[1] = transform.transform.translation.y;
        position[2] = transform.transform.translation.z;

        return true;  // 查询成功

    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "手动查询TF失败: %s", ex.what());
        return false;  // 查询失败
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<PctPlanner>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
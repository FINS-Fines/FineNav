#include "rclcpp/rclcpp.hpp"
#include "fn_fine_pct/Tomography.hpp"
#include <memory>

/**
 * New Terminal:
 * source /opt/ros/humble/setup.bash
 * =================================
 * Enter Workspace:
 * colcon build
 * source install/setup.bash
 * =================================
 * ros2 run fn_fine_pct tomography_node
 * ros2 run fn_fine_pct tomography_node --ros-args -p pcd_file_path:=rsc/another.pcd
 * =================================
 * ros2 run rviz2 rviz2
 * =================================
 * pcl_viewer building.pcd
 */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto tomography_node = std::make_shared<Tomography>();

    // // 可选：短暂延迟确保消息发布完成
    // rclcpp::sleep_for(std::chrono::seconds(1));

    rclcpp::shutdown();

    return 0;
}
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
 */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto tomography_node = std::make_shared<Tomography>();

    rclcpp::spin(tomography_node);
    rclcpp::shutdown();

    return 0;
}
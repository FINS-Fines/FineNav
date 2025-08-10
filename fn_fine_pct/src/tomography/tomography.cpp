#include "fn_fine_pct/Tomography.hpp"
#include "fn_fine_pct/Config.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

Tomography::Tomography()
    : Node("pointcloud_tomography"),
      pcd_file_path_("rsc/pcd/building.pcd"),  // 硬编码或通过参数获取
      cloud_(new pcl::PointCloud<pcl::PointXYZ>) {

    this->declare_parameter("pcd_file_path", pcd_file_path_);

    pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "processed_pointcloud", 10);

    loadPCD();
    processPointCloud();
}

void Tomography::loadPCD() {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s",
                    pcd_file_path_.c_str());
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded PCD file: %s with %ld points",
               pcd_file_path_.c_str(), cloud_->size());
}

void Tomography::processPointCloud() {
    RCLCPP_INFO(this->get_logger(), "START::Tomography");
    // tomography algorithm

    // 示例：发布处理后的点云
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_, output);
    pub_->publish(output);
}
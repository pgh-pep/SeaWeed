#include "seaweed_perception/surface_scan_node.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using namespace seaweed_perception;

SurfaceScanNode::SurfaceScanNode() : rclcpp::Node("water_surface_laser_scan") {
    pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10,
        std::bind(&SurfaceScanNode::point_cloud_callback, this, std::placeholders::_1));

    laser_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    debug_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/pointcloud", 10);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

}

void SurfaceScanNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2::SharedPtr transformed_pc = std::make_shared<sensor_msgs::msg::PointCloud2>();

    if (!transform_to_base_link(msg, transformed_pc)){
        RCLCPP_ERROR(this->get_logger(), "Failed to transform point cloud");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    debug_pointcloud(pcl_cloud);
    
}

bool SurfaceScanNode::transform_to_base_link(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                                             sensor_msgs::msg::PointCloud2::SharedPtr& output_cloud) {
    try {
        std::string target_frame = "/base_link";

        // validate transform to base_link exists
        if (!tf_buffer->canTransform(target_frame, input_cloud->header.frame_id, input_cloud->header.stamp,
                                     rclcpp::Duration::from_seconds(1.0))) {
            RCLCPP_WARN(this->get_logger(), "Transform from %s to %s not available", input_cloud->header.frame_id.c_str(),
                        target_frame.c_str());
            return false;
        }

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer->lookupTransform(target_frame, input_cloud->header.frame_id, input_cloud->header.stamp,
                                                       rclcpp::Duration::from_seconds(1.0));

        tf2::doTransform(*input_cloud, *output_cloud, transform_stamped);
        output_cloud->header.frame_id = target_frame;

        return true;

    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
        return false;
    }
}

void SurfaceScanNode::debug_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
    sensor_msgs::msg::PointCloud2 debug_pc_msg;
    pcl::toROSMsg(*pcl_cloud, debug_pc_msg);

    debug_pc_msg.header.stamp = this->get_clock()->now();
    debug_pc_msg.header.frame_id = "base_link";

    debug_pointcloud_pub->publish(debug_pc_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published debug point cloud with %zu points", pcl_cloud->size());
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto surface_scan_node = std::make_shared<SurfaceScanNode>();

    RCLCPP_INFO(surface_scan_node->get_logger(), "Starting Surface scan node: ");

    rclcpp::spin(surface_scan_node);
    rclcpp::shutdown();
    return 0;
}

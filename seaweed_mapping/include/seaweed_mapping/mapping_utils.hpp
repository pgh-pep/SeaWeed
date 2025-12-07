#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>

namespace mapping_utils {

void ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& pc_msg,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl);

void pcl_to_ros(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl,
                sensor_msgs::msg::PointCloud2::SharedPtr& pc_msg, const std::string& target_frame,
                rclcpp::Clock::SharedPtr clock);

void transform_pc(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_pc,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_pc, const std::string& target_frame,
                  std::shared_ptr<tf2_ros::Buffer> tf_buffer, rclcpp::Logger logger);

void debug_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, const std::string& target_frame,
                      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                      rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);

void create_marker(float x, float y, float z, int id, const std::string& frame,
                   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
                   const std::string& label = "");

void reset_markers(const std::string& frame,
                   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher);

}  // namespace mapping_utils
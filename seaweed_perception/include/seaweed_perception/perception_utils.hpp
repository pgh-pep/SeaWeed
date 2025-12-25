#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace perception_utils {

enum Color { RED, GREEN, BLUE };
struct Point {
    float x;
    float y;
};

struct Detection {
    Point point;
    rclcpp::Time timestamp;
    int num_detections;
};

std_msgs::msg::ColorRGBA get_rgba_color(Color color, float alpha = 1.0f);

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

void create_marker(const float& x, const float& y, const float& z, const int& id, const std::string& frame,
                   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher, const Color& color,
                   const std::string& label = "");

void reset_markers(const std::string& frame,
                   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher);

}  // namespace perception_utils
#ifndef WATER_SURFACE_LASER_SCAN_HPP
#define WATER_SURFACE_LASER_SCAN_HPP

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace seaweed_perception {

class SurfaceScanNode : public rclcpp::Node {
public:
    SurfaceScanNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_pub;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    double z_min;
    double z_max;
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
    double scan_time;
    std::string frame_id;

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    bool transform_to_base_link(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                                sensor_msgs::msg::PointCloud2::SharedPtr& output_cloud);

    void debug_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);

    sensor_msgs::msg::LaserScan convert_to_scan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                const std_msgs::msg::Header& header);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_water_surface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
};

}  // namespace seaweed_perception

#endif
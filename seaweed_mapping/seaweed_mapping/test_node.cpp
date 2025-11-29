#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node {
public:
    TestNode() : Node("test_node") {
        pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10,
            std::bind(&TestNode::pc_callback, this, std::placeholders::_1));

        debug_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/pointcloud", 10);

        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        RCLCPP_INFO(this->get_logger(), "started test node");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_pub;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    const std::string base_link = "wamv/base_link";

    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZ>);

        ros_to_pcl(msg, pc);
        transform_pc(pc, transformed_pc, base_link);
        debug_pointcloud(transformed_pc, base_link);
    }

    void ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& pc_msg,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl) {
        pcl::fromROSMsg(*pc_msg, *pc_pcl);
    }

    void pcl_to_ros(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl,
                    sensor_msgs::msg::PointCloud2::SharedPtr& pc_msg, const std::string target_frame) {
        pcl::toROSMsg(*pc_pcl, *pc_msg);
        pc_msg->header.stamp = this->get_clock()->now();
        pc_msg->header.frame_id = target_frame;
    }

    void transform_pc(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_pc,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_pc, const std::string& target_frame) {
        try {
            geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer->lookupTransform(
                target_frame, original_pc->header.frame_id, rclcpp::Time{}, rclcpp::Duration::from_seconds(1.0));

            pcl_ros::transformPointCloud(*original_pc, *transformed_pc, tf_stamped);
            transformed_pc->header.frame_id = target_frame;

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "FAILED PC TRANSFORM: %s", ex.what());
        }
    }

    void debug_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, const std::string& target_frame) {
        sensor_msgs::msg::PointCloud2::SharedPtr debug_pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl_to_ros(pc, debug_pc_msg, target_frame);
        debug_pc_msg->header.frame_id = target_frame;

        debug_pointcloud_pub->publish(*debug_pc_msg);

        RCLCPP_INFO(this->get_logger(), "pub debug pc w/ %zu points", pc->size());
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}
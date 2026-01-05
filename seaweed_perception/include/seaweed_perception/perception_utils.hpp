#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "seaweed_interfaces/msg/labeled_pose_array.hpp"
namespace perception_utils {

enum Color { RED, GREEN, BLUE };
struct Point {
    float x;
    float y;
    float z;
};

std::ostream& operator<<(std::ostream& os, const Point& p);

struct Detection {
    Point point;
    rclcpp::Time timestamp;
    int num_detections;
};

struct LabeledDetection {
    std::string label;
    Detection detection;
};

struct CameraIntrinsics {
    float fx, fy, cx, cy;

    void set_from_cam_info(const sensor_msgs::msg::CameraInfo& msg) {
        fx = msg.k[0];
        fy = msg.k[4];
        cx = msg.k[2];
        cy = msg.k[5];
    }

    std::array<float, 3> project_to_3d(int u, int v, float depth) const {
        float x_c = (u - cx) * depth / fx;
        float y_c = (v - cy) * depth / fy;
        float z_c = depth;
        return {x_c, y_c, z_c};
    }
};

// same as rclcpp::SensorDataQoS();
static const rclcpp::QoS image_qos = rclcpp::QoS(5)
                                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                                         .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

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
                   std::string ns, const Color& color, const std::string& label,
                   std::vector<visualization_msgs::msg::Marker>& markers);

void reset_markers(const std::string& frame, std::string ns,
                   std::vector<visualization_msgs::msg::Marker>& markers);

bool transform_labeled_pose(const seaweed_interfaces::msg::LabeledPose& original_pose,
                            seaweed_interfaces::msg::LabeledPose& transformed_pose,
                            const std::string& target_frame, const std::string& source_frame,
                            std::shared_ptr<tf2_ros::Buffer> tf_buffer, const rclcpp::Logger& logger);

bool transform_labeled_pose_array(const seaweed_interfaces::msg::LabeledPoseArray& original_pose_array,
                                  seaweed_interfaces::msg::LabeledPoseArray& transformed_pose_array,
                                  const std::string& target_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                  const rclcpp::Logger& logger);

bool transform_pose(const geometry_msgs::msg::PoseStamped& original_pose,
                    geometry_msgs::msg::PoseStamped& transformed_pose, const std::string& target_frame,
                    std::shared_ptr<tf2_ros::Buffer> tf_buffer, const rclcpp::Logger& logger);

bool transform_pose_array(const geometry_msgs::msg::PoseArray& original_pose_array,
                          geometry_msgs::msg::PoseArray& transformed_pose_array, const std::string& target_frame,
                          std::shared_ptr<tf2_ros::Buffer> tf_buffer, const rclcpp::Logger& logger);

}  // namespace perception_utils
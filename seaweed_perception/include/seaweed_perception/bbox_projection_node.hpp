#pragma once

#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "seaweed_interfaces/msg/bounding_box.hpp"
#include "seaweed_interfaces/msg/detection.hpp"
#include "seaweed_interfaces/msg/labeled_pose.hpp"
#include "seaweed_interfaces/msg/labeled_pose_array.hpp"
#include "seaweed_perception/perception_utils.hpp"

// NOTE: assumes no distortion (ex. gazebo or zed 2i), add flag for distortion
class BBox_Projection_Node : public rclcpp::Node {
public:
    BBox_Projection_Node();

private:
    std::string rgb_image_topic;
    std::string depth_image_topic;
    std::string camera_info_topic;
    std::string detection_topic;
    std::string projection_topic;

    std::string base_link_frame;
    std::string map_frame;
    std::string camera_optical_frame;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    rclcpp::Subscription<seaweed_interfaces::msg::Detection>::SharedPtr detection_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr projection_pub;

    cv::Mat latest_rgb_image;
    cv::Mat latest_depth_image;
    cv_bridge::CvImagePtr cv_ptr;
    rclcpp::Time camera_update_timestamp;
    bool recieved_img;
    bool intrinsics_set;
    float image_expiration_threshold;

    perception_utils::CameraIntrinsics camera_intrinsics;

    std::vector<seaweed_interfaces::msg::BoundingBox> detections;
    seaweed_interfaces::msg::LabeledPoseArray bbox_projections;

    std::vector<perception_utils::Point> depth_sample_points;
    std::vector<float> depths;
    float sample_scaling_factor;
    float mad_threshold_multiplier;

    void rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void detection_callback(const seaweed_interfaces::msg::Detection::SharedPtr msg);

    bool is_image_valid(const rclcpp::Time timestamp, float expiration_seconds);

    float get_depth_at_pixel(const int u, const int v);
    float sample_depth(std::vector<perception_utils::Point>& samples, std::vector<float>& _depths, const int u,
                       const int v, const int bbox_width, const int bbox_height, const float scaling_factor);

    float calc_median(std::vector<float>& values);
    float calc_med_abs_dev_threshold(std::vector<float> values, float median, float _threshold_multiplier);

    void visualize_pose_array(const geometry_msgs::msg::PoseArray& pose_array,
                              const std::vector<std::string>& labels);
};
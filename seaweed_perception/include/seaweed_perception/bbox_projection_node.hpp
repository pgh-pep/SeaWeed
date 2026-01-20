#pragma once

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "seaweed_interfaces/msg/bounding_box.hpp"
#include "seaweed_interfaces/msg/detection.hpp"
#include "seaweed_interfaces/msg/labeled_pose.hpp"
#include "seaweed_interfaces/msg/labeled_pose_array.hpp"
#include "seaweed_perception/hungarian.hpp"
#include "seaweed_perception/perception_utils.hpp"

struct ClusterProjection {
    int u, v;
    float depth_c;                  // cam frame
    geometry_msgs::msg::Point pos;  // original pose in cluster frame
    bool matched = false;
};

class BBox_Projection_Node : public rclcpp::Node {
public:
    BBox_Projection_Node();

private:
    void cluster_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void yolo_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void detection_callback(const seaweed_interfaces::msg::Detection::SharedPtr msg);

    void clusters_to_projections(const geometry_msgs::msg::PoseArray& clusters,
                                 std::vector<ClusterProjection>& cluster_projections, rclcpp::Time stamp);

    int find_cluster_in_bbox(const seaweed_interfaces::msg::BoundingBox& bbox,
                             std::vector<ClusterProjection>& projections);

    bool bbox_to_pose_c(const seaweed_interfaces::msg::BoundingBox& bbox, geometry_msgs::msg::Pose& pose_c);

    float sample_depth(int u, int v, int bbox_width, int bbox_height);
    float get_depth_at_pixel(int u, int v);
    float calc_median(std::vector<float>& values);
    float calc_mad_threshold(std::vector<float> values, float median);
    bool is_data_valid(const rclcpp::Time& timestamp);
    void debug_markers(const seaweed_interfaces::msg::LabeledPoseArray& matched_poses,
                       const seaweed_interfaces::msg::LabeledPoseArray& unmatched_bbox_poses,
                       const geometry_msgs::msg::PoseArray& unmatched_cluster_poses);

    //    ADDED
    float calc_cost(const seaweed_interfaces::msg::BoundingBox& bbox, const ClusterProjection& proj);
    void match_detections_to_clusters(const std::vector<seaweed_interfaces::msg::BoundingBox>& bboxes,
                                      std::vector<ClusterProjection>& projections,
                                      std::vector<std::pair<int, int>>& matches_idx,
                                      std::vector<int>& unmatched_bboxes_idx,
                                      std::vector<int>& unmatched_clusters_idx);

    void visualize_on_img(const std::vector<ClusterProjection>& projections);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    rclcpp::Subscription<seaweed_interfaces::msg::Detection>::SharedPtr detection_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cluster_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Publisher<seaweed_interfaces::msg::LabeledPoseArray>::SharedPtr matched_pub;
    rclcpp::Publisher<seaweed_interfaces::msg::LabeledPoseArray>::SharedPtr unmatched_bbox_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr unmatched_cluster_pub;

    bool debug;

    std::string depth_image_topic;
    std::string yolo_bbox_image_topic;

    std::string camera_info_topic;
    std::string detection_topic;
    std::string cluster_topic;
    std::string matched_topic;
    std::string unmatched_bbox_topic;
    std::string unmatched_cluster_topic;
    std::string base_link_frame;
    std::string map_frame;
    std::string camera_optical_frame;

    cv::Mat latest_depth_image;
    geometry_msgs::msg::PoseArray latest_clusters;
    perception_utils::CameraIntrinsics camera_intrinsics;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr yolo_debug_image_sub;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub;

    cv::Mat latest_yolo_debug_image;
    bool recieved_yolo;

    rclcpp::Time depth_update_stamp;
    rclcpp::Time cluster_update_stamp;

    bool recieved_depth;
    bool recieved_clusters;
    bool intrinsics_set;

    float expiration_thresh;
    float sample_scaling_factor;
    float mad_threshold_multiplier;
    float max_assignment_cost;
    float bbox_margin;
};

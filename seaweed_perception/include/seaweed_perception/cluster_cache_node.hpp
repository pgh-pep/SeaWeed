#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "seaweed_perception/perception_utils.hpp"

class ClusterCacheNode : public rclcpp::Node {
public:
    ClusterCacheNode();

private:
    std::string cluster_topic, cache_topic, base_link_frame, map_frame;
    std::vector<perception_utils::Detection> detections_cache;

    float same_cluster_dist_threshold, detection_expiration_threshold;
    bool debug;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cluster_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cache_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

    rclcpp::TimerBase::SharedPtr cache_timer;

    void cluster_callback(const geometry_msgs::msg::PoseArray& msg);

    float euclidian_distance(const perception_utils::Point& p0, const perception_utils::Point& p1);

    bool check_expired(const perception_utils::Detection& detection, float expiration_seconds);

    void publish_debug_markers();

    void publish_cache();
};

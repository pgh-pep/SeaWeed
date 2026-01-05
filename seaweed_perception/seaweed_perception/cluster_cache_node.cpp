#include "seaweed_perception/cluster_cache_node.hpp"

ClusterCacheNode::ClusterCacheNode()
    : Node("cluster_cache"),
      cluster_topic("/debug/clusters"),
      cache_topic("/cluster_cache"),
      base_link_frame("wamv/base_link"),
      same_cluster_dist_threshold(.25),
      detection_expiration_threshold(5),
      debug(true) {
    cluster_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        cluster_topic, 10, std::bind(&ClusterCacheNode::cluster_callback, this, std::placeholders::_1));
    cache_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(cache_topic, 10);
    cache_timer = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(100),
                                       std::bind(&ClusterCacheNode::publish_cache, this));
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/markers", 1);
}

// NOTE: we should have a semantic obstacle list for each event
// add client service to remove points when fused

void ClusterCacheNode::cluster_callback(const geometry_msgs::msg::PoseArray& msg) {
    bool is_existing_detection = false;

    for (const auto& pose : msg.poses) {
        perception_utils::Point detected_point = {(float)pose.position.x, (float)pose.position.y,
                                                  (float)pose.position.z};
        is_existing_detection = false;
        for (perception_utils::Detection& old_detection : detections_cache) {
            if (euclidian_distance(old_detection.point, detected_point) < same_cluster_dist_threshold) {
                // if within threshold, must be the same point
                // TODO: IMPLEMENT LOCATION AVERAGING
                old_detection.num_detections += 1;
                is_existing_detection = true;
                old_detection.timestamp = this->get_clock()->now();
                // RCLCPP_INFO(this->get_logger(), "old cluster, x: %f, y: %f", old_detection.point.x,
                //             old_detection.point.y);

                break;
            }
        }
        // else, must be a new point
        if (!is_existing_detection) {
            // RCLCPP_INFO(this->get_logger(), "new cluster, x: %f, y: %f", detected_point.x, detected_point.y);
            perception_utils::Detection new_detection;
            new_detection.point = detected_point;
            new_detection.timestamp = this->get_clock()->now();
            new_detection.num_detections = 1;
            detections_cache.push_back(new_detection);
        }
    }

    // REMOVE OLD POINTS
    for (auto it = detections_cache.begin(); it != detections_cache.end();) {
        if (check_expired(*it, detection_expiration_threshold)) {
            RCLCPP_INFO(this->get_logger(), "REMOVED FROM CACHE");
            it = detections_cache.erase(it);
            // TODO: service call to publish to map as unknown
        } else {
            ++it;
        }
    }
}

float ClusterCacheNode::euclidian_distance(const perception_utils::Point& p0, const perception_utils::Point& p1) {
    return std::hypot(p0.x - p1.x, p0.y - p1.y);
}

bool ClusterCacheNode::check_expired(const perception_utils::Detection& detection, float expiration_seconds) {
    rclcpp::Duration last_detected_time = this->get_clock()->now() - detection.timestamp;
    if (last_detected_time.seconds() > expiration_seconds) {
        return true;
    }
    return false;
}

void ClusterCacheNode::publish_debug_markers() {
    visualization_msgs::msg::MarkerArray marker_array;
    perception_utils::reset_markers(base_link_frame, "cluster_cache", marker_array.markers);

    int i = 0;
    for (const auto& detection : detections_cache) {
        perception_utils::create_marker(detection.point.x, detection.point.y, detection.point.z, i,
                                        base_link_frame, "cluster_cache", perception_utils::Color::RED, "cache",
                                        marker_array.markers);
        i++;
    }
    marker_pub->publish(marker_array);
}

void ClusterCacheNode::publish_cache() {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = base_link_frame;

    for (const auto& detection : detections_cache) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = detection.point.x;
        pose.position.y = detection.point.y;
        pose.position.z = detection.point.z;
        msg.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "cache size: %zu", msg.poses.size());

    if (debug) {
        publish_debug_markers();
    }
    cache_pub->publish(msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterCacheNode>());
    rclcpp::shutdown();
    return 0;
}
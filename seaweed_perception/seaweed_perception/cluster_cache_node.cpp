#include "seaweed_perception/cluster_cache_node.hpp"

ClusterCacheNode::ClusterCacheNode()
    : Node("cluster_cache"),
      cluster_topic("/debug/clusters"),
      cache_topic("/cluster_cache"),
      cluster_frame("wamv/base_link"),
      same_cluster_dist_threshold(.25),
      detection_expiration_threshold(5) {
    cluster_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        cluster_topic, 10, std::bind(&ClusterCacheNode::cluster_callback, this, std::placeholders::_1));
    cache_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(cache_topic, 10);
    cache_timer = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(100),
                                       std::bind(&ClusterCacheNode::publish_cache, this));
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/markers", 1);
}

void ClusterCacheNode::cluster_callback(const geometry_msgs::msg::PoseArray& msg) {
    // add frame validation perchance?
    bool is_existing_detection = false;

    for (const auto& pose : msg.poses) {
        perception_utils::Point detected_point = {(float)pose.position.x, (float)pose.position.y};
        is_existing_detection = false;
        for (perception_utils::Detection& old_detection : detections_cache) {
            if (euclidian_distance(old_detection.point, detected_point) < same_cluster_dist_threshold) {
                // if within threshold, must be the same point
                // TODO: IMPLEMENT LOCATION AVERAGING
                old_detection.num_detections += 1;
                is_existing_detection = true;
                old_detection.timestamp = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "old cluster, x: %f, y: %f", old_detection.point.x,
                            old_detection.point.y);

                break;
            }
        }
        if (!is_existing_detection) {  // else, must be a new point
            RCLCPP_INFO(this->get_logger(), "new cluster, x: %f, y: %f", detected_point.x, detected_point.y);
            perception_utils::Detection new_detection;
            new_detection.point = detected_point;
            new_detection.timestamp = this->get_clock()->now();
            new_detection.num_detections = 1;
            detections_cache.push_back(new_detection);
        }
    }

    // Check if any detection has not been updated recently
    for (auto it = detections_cache.begin(); it != detections_cache.end();) {
        if (check_expired(*it, detection_expiration_threshold)) {
            // TODO: if so,  publish to map as expired/unlabled
            RCLCPP_INFO(this->get_logger(), "REMOVED FROM CACHE");

            // remove from cache
            it = detections_cache.erase(it);
        } else {
            ++it;
        }
    }
    // Note that even if an object is unlabeled, it should still appear in the semantic one as an unlabled
    // object also, we should have a semantic obstacle list for each event
}

float ClusterCacheNode::euclidian_distance(const perception_utils::Point& p0, const perception_utils::Point& p1) {
    return std::hypot(p0.x - p1.x, p0.y - p1.y);
}

bool ClusterCacheNode::check_expired(const perception_utils::Detection detection, float expiration_seconds) {
    rclcpp::Duration last_detected_time = this->get_clock()->now() - detection.timestamp;
    if (last_detected_time.seconds() > expiration_seconds) {
        return true;
    }
    return false;
}

void ClusterCacheNode::publish_cache() {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = cluster_frame;

    visualization_msgs::msg::MarkerArray marker_array;
    perception_utils::reset_markers(cluster_frame, marker_array.markers);

    int i = 0;
    for (const auto& detection : detections_cache) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = detection.point.x;
        pose.position.y = detection.point.y;
        msg.poses.push_back(pose);

        perception_utils::create_marker(detection.point.x, detection.point.y, 0.0, i, cluster_frame,
                                        perception_utils::Color::RED, "cluster_cache", marker_array.markers);
        i++;
    }

    // marker_pub->publish(marker_array);
    cache_pub->publish(msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterCacheNode>());
    rclcpp::shutdown();
    return 0;
}
#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "seaweed_perception/perception_utils.hpp"

class ClusterCache : public rclcpp::Node {
public:
    ClusterCache()
        : Node("cluster_cache"),
          cluster_topic("/debug/clusters"),
          cluster_frame("/odom"),
          same_cluster_dist_threshold(.25),
          detection_expiration_threshold(5) {
        cluster_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
            cluster_topic, 10, std::bind(&ClusterCache::cluster_callback, this, std::placeholders::_1));
    };

private:
    std::string cluster_topic;
    std::string cluster_frame;
    std::vector<perception_utils::Detection> detections_cache;

    rclcpp::Time current_time;

    float same_cluster_dist_threshold;
    float detection_expiration_threshold;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cluster_sub;

    void cluster_callback(const geometry_msgs::msg::PoseArray& msg) {
        // add frame validation perchance?
        perception_utils::Detection new_detection;
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

                    continue;
                }
            }
            if (!is_existing_detection) {  // else, must be a new point
                RCLCPP_INFO(this->get_logger(), "new cluster, x: %f, y: %f", detected_point.x, detected_point.y);
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
    };

    float euclidian_distance(const perception_utils::Point& p0, const perception_utils::Point& p1) {
        return std::hypot(p0.x - p1.x, p0.y - p1.y);
    };

    bool check_expired(const perception_utils::Detection detection, float expiration_seconds) {
        rclcpp::Duration last_detected_time = this->get_clock()->now() - detection.timestamp;
        if (last_detected_time.seconds() > expiration_seconds) {
            return true;
        }
        return false;
    };
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterCache>());
    rclcpp::shutdown();
    return 0;
}
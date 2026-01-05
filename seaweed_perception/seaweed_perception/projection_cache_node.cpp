#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "seaweed_interfaces/msg/labeled_pose.hpp"
#include "seaweed_interfaces/msg/labeled_pose_array.hpp"
#include "seaweed_perception/perception_utils.hpp"

class ProjectionCacheNode : public rclcpp::Node {
public:
    ProjectionCacheNode()
        : Node("projection_cache_node"),
          projection_topic("/bbox_projection"),
          base_link_frame("wamv/base_link"),
          map_frame("map"),
          projection_cache_topic("/projection_cache"),
          projection_frame("wamv/base_link"),
          same_projection_dist_threshold(.25),
          projection_expiration_threshold(5),
          next_detection_id(0),
          debug(false) {
        projection_sub = this->create_subscription<seaweed_interfaces::msg::LabeledPoseArray>(
            projection_topic, 10,
            std::bind(&ProjectionCacheNode::projection_callback, this, std::placeholders::_1));
        projection_cache_pub =
            this->create_publisher<seaweed_interfaces::msg::LabeledPoseArray>(projection_cache_topic, 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/markers", 1);
        cache_timer = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(100),
                                           std::bind(&ProjectionCacheNode::publish_cache, this));
    }

private:
    std::string projection_topic, base_link_frame, map_frame, projection_cache_topic, projection_frame;
    float same_projection_dist_threshold, projection_expiration_threshold;
    uint32_t next_detection_id;
    bool debug;

    rclcpp::Subscription<seaweed_interfaces::msg::LabeledPoseArray>::SharedPtr projection_sub;
    rclcpp::Publisher<seaweed_interfaces::msg::LabeledPoseArray>::SharedPtr projection_cache_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

    rclcpp::TimerBase::SharedPtr cache_timer;

    // figure out exact strucuture of cache
    std::vector<perception_utils::LabeledDetection> projection_cache, mapped_projections;

    void projection_callback(const seaweed_interfaces::msg::LabeledPoseArray& msg) {
        for (const auto& l_pose : msg.labeled_poses) {
            perception_utils::Point detected_point = {(float)l_pose.pose.position.x, (float)l_pose.pose.position.y,
                                                      (float)l_pose.pose.position.z};

            // find the closest cached projection within threshold w/ matching label
            float min_dist = same_projection_dist_threshold;
            perception_utils::LabeledDetection* closest_projection = nullptr;

            for (perception_utils::LabeledDetection& old_projection : projection_cache) {
                if (old_projection.label == l_pose.label) {
                    float dist = euclidian_distance(old_projection.detection.point, detected_point);
                    if (dist < min_dist) {
                        min_dist = dist;
                        closest_projection = &old_projection;
                    }
                }
            }

            if (closest_projection) {
                // if within threshold, must be the same point
                // average location with old projection:
                closest_projection->detection.point.x =
                    (closest_projection->detection.point.x * closest_projection->detection.num_detections +
                     detected_point.x) /
                    (closest_projection->detection.num_detections + 1);
                closest_projection->detection.point.y =
                    (closest_projection->detection.point.y * closest_projection->detection.num_detections +
                     detected_point.y) /
                    (closest_projection->detection.num_detections + 1);
                closest_projection->detection.point.z =
                    (closest_projection->detection.point.z * closest_projection->detection.num_detections +
                     detected_point.z) /
                    (closest_projection->detection.num_detections + 1);

                closest_projection->detection.num_detections += 1;
                closest_projection->detection.timestamp = this->get_clock()->now();
                // RCLCPP_INFO(this->get_logger(), "old projection, x: %f, y: %f", old_detection.point.x,
                //             old_detection.point.y);
            } else {
                // else, must be a new point
                // RCLCPP_INFO(this->get_logger(), "new projection, x: %f, y: %f", detected_point.x,
                // detected_point.y);
                perception_utils::LabeledDetection new_l_detection;
                new_l_detection.detection.point = detected_point;
                new_l_detection.detection.timestamp = this->get_clock()->now();
                new_l_detection.detection.num_detections = 1;
                new_l_detection.detection.id = next_detection_id++;
                new_l_detection.label = l_pose.label;
                projection_cache.push_back(new_l_detection);
            }
        }

        // REMOVE OLD POINTS
        for (auto it = projection_cache.begin(); it != projection_cache.end();) {
            if (check_expired(*it, projection_expiration_threshold)) {
                RCLCPP_INFO(this->get_logger(), "REMOVED FROM PROJ CACHE");
                it = projection_cache.erase(it);
                // TODO: service call to publish to map as unknown
            } else {
                ++it;
            }
        }
    };

    void publish_cache() {
        seaweed_interfaces::msg::LabeledPoseArray msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = base_link_frame;

        for (const auto& projection : projection_cache) {
            seaweed_interfaces::msg::LabeledPose l_pose;
            l_pose.label = projection.label;
            l_pose.pose.position.x = projection.detection.point.x;
            l_pose.pose.position.y = projection.detection.point.y;
            l_pose.pose.position.z = projection.detection.point.z;
            l_pose.id = projection.detection.id;
            msg.labeled_poses.push_back(l_pose);
        }

        RCLCPP_INFO(this->get_logger(), "proj cache size: %zu", msg.labeled_poses.size());

        if (debug) {
            publish_debug_markers();
        }
        projection_cache_pub->publish(msg);
    }

    float euclidian_distance(const perception_utils::Point& p0, const perception_utils::Point& p1) {
        return std::hypot(p0.x - p1.x, p0.y - p1.y);
    };

    bool check_expired(const perception_utils::LabeledDetection& l_detection, float expiration_seconds) {
        rclcpp::Duration last_detected_time = this->get_clock()->now() - l_detection.detection.timestamp;
        if (last_detected_time.seconds() > expiration_seconds) {
            return true;
        }
        return false;
    };

    void publish_debug_markers() {
        visualization_msgs::msg::MarkerArray marker_array;
        perception_utils::reset_markers(base_link_frame, "projection_cache", marker_array.markers);

        int i = 0;
        for (const auto& projection : projection_cache) {
            perception_utils::create_marker(projection.detection.point.x, projection.detection.point.y,
                                            projection.detection.point.z, i, base_link_frame, "projection_cache",
                                            perception_utils::Color::BLUE, projection.label, marker_array.markers);
            i++;
        }
        marker_pub->publish(marker_array);
    };
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProjectionCacheNode>());
    rclcpp::shutdown();
    return 0;
};
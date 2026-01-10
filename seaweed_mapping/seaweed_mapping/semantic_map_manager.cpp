#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "seaweed_interfaces/msg/labeled_pose.hpp"
#include "seaweed_interfaces/msg/labeled_pose_array.hpp"
#include "seaweed_perception/perception_utils.hpp"

class SemanticMapManager : public rclcpp::Node {
public:
    SemanticMapManager()
        : Node("semantic_map_manager"),
          debug(true),
          merge_thresh(1.0),
          min_observ_threshold(2),
          matched_obstacles_topic("/mapping/matched"),
          unmatched_bbox_topic("/mapping/unmatched_bbox"),
          unmatched_clusters_topic("/mapping/unmatched_clusters"),
          semantic_map_topic("/mapping/semantic_map"),
          debug_vis_topic("/debug/semantic_map_markers"),
          map_frame("map") {
        matched_sub = this->create_subscription<seaweed_interfaces::msg::LabeledPoseArray>(
            matched_obstacles_topic, 10,
            std::bind(&SemanticMapManager::matched_callback, this, std::placeholders::_1));

        unmatched_bbox_sub = this->create_subscription<seaweed_interfaces::msg::LabeledPoseArray>(
            unmatched_bbox_topic, 10,
            std::bind(&SemanticMapManager::unmatched_bbox_callback, this, std::placeholders::_1));

        unmatched_cluster_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
            unmatched_clusters_topic, 10,
            std::bind(&SemanticMapManager::unmatched_cluster_callback, this, std::placeholders::_1));

        semantic_map_pub =
            this->create_publisher<seaweed_interfaces::msg::LabeledPoseArray>(semantic_map_topic, 10);

        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(debug_vis_topic, 10);

        reset_srv = this->create_service<std_srvs::srv::Trigger>(
            "/mapping/semantic/reset",
            std::bind(&SemanticMapManager::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

        map_publish_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                    std::bind(&SemanticMapManager::publish_semantic_map, this));
    }

private:
    struct TrackedObstacle {
        seaweed_interfaces::msg::LabeledPose l_pose;
        int detection_count;
        rclcpp::Time last_seen;
    };

    bool debug;

    float merge_thresh;
    int min_observ_threshold;

    std::string matched_obstacles_topic;
    std::string unmatched_bbox_topic;
    std::string unmatched_clusters_topic;
    std::string semantic_map_topic;
    std::string debug_vis_topic;
    std::string map_frame;

    std::vector<TrackedObstacle> semantic_map;

    rclcpp::Subscription<seaweed_interfaces::msg::LabeledPoseArray>::SharedPtr matched_sub;
    rclcpp::Subscription<seaweed_interfaces::msg::LabeledPoseArray>::SharedPtr unmatched_bbox_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr unmatched_cluster_sub;
    rclcpp::Publisher<seaweed_interfaces::msg::LabeledPoseArray>::SharedPtr semantic_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;

    rclcpp::TimerBase::SharedPtr map_publish_timer;

    void matched_callback(const seaweed_interfaces::msg::LabeledPoseArray::SharedPtr msg) {
        for (const auto& labeled_pose : msg->labeled_poses) {
            process_obstacle(labeled_pose);
        }
    }

    void unmatched_bbox_callback(const seaweed_interfaces::msg::LabeledPoseArray::SharedPtr msg) {
        for (const auto& labeled_pose : msg->labeled_poses) {
            process_obstacle(labeled_pose);
        }
    }

    void unmatched_cluster_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        for (const auto& pose : msg->poses) {
            seaweed_interfaces::msg::LabeledPose labeled_pose;
            labeled_pose.label = "UNKNOWN";
            labeled_pose.pose = pose;
            process_obstacle(labeled_pose);
        }
    }

    void reset_callback(const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
        size_t n = semantic_map.size();
        semantic_map.clear();
        response->success = true;
        response->message = "Cleared " + std::to_string(n) + " obstacles";
        RCLCPP_INFO(this->get_logger(), "RESET SEMANTIC MAP: %zu obstacles", n);
    }

    void process_obstacle(const seaweed_interfaces::msg::LabeledPose& new_pose) {
        TrackedObstacle* match = find_match(new_pose);
        if (match) {
            update_obstacle(*match, new_pose);
        } else {
            add_obstacle(new_pose);
        }
    }

    TrackedObstacle* find_match(const seaweed_interfaces::msg::LabeledPose& pose) {
        TrackedObstacle* unknown_match = nullptr;

        for (auto& obstacle : semantic_map) {
            // diff obstacles if sufficient distance apart
            if (eucl_distance(obstacle.l_pose.pose.position, pose.pose.position) >= merge_thresh) {
                continue;
            }

            // else: same obstacle, check if labeled-labeled pair:
            if (obstacle.l_pose.label == pose.label) {
                return &obstacle;
            }

            // else, check if unknown-labeled pair
            bool is_unknown_incoming = (pose.label == "UNKNOWN");
            bool is_unknown_mapped = (obstacle.l_pose.label == "UNKNOWN");

            if (is_unknown_incoming != is_unknown_mapped) {
                unknown_match = &obstacle;
                // note: currently does not return here to check
                // if labeled obj within thresh exists (prioritize labeled)
            }
        }

        // else, must be unknown-unknown pair
        return unknown_match;
    }

    void update_obstacle(TrackedObstacle& obstacle, const seaweed_interfaces::msg::LabeledPose& new_pose) {
        int n = obstacle.detection_count;
        geometry_msgs::msg::Point& pos = obstacle.l_pose.pose.position;
        const auto& new_pos = new_pose.pose.position;

        pos.x = (pos.x * n + new_pos.x) / (n + 1);
        pos.y = (pos.y * n + new_pos.y) / (n + 1);
        pos.z = (pos.z * n + new_pos.z) / (n + 1);

        // replace unknown with labeled:
        if (obstacle.l_pose.label == "UNKNOWN" && new_pose.label != "UNKNOWN") {
            obstacle.l_pose.label = new_pose.label;
        }

        obstacle.detection_count++;
        obstacle.last_seen = this->get_clock()->now();
    }

    void add_obstacle(const seaweed_interfaces::msg::LabeledPose& l_pose) {
        TrackedObstacle new_obstacle;
        new_obstacle.l_pose = l_pose;
        new_obstacle.detection_count = 1;
        new_obstacle.last_seen = this->get_clock()->now();
        semantic_map.push_back(new_obstacle);
    }

    float eucl_distance(const geometry_msgs::msg::Point& p0, const geometry_msgs::msg::Point& p1) {
        return std::hypot(p1.x - p0.x, p1.y - p0.y);
    }

    void publish_semantic_map() {
        seaweed_interfaces::msg::LabeledPoseArray msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = map_frame;

        for (const auto& obstacle : semantic_map) {
            // TEST if good idea: only pub if detected multiple times
            if (obstacle.detection_count >= min_observ_threshold) {
                msg.labeled_poses.push_back(obstacle.l_pose);
            }
        }

        semantic_map_pub->publish(msg);
        if (debug) {
            visualize_map_markers();
        }
    }

    void visualize_map_markers() {
        visualization_msgs::msg::MarkerArray markers;
        perception_utils::reset_markers(map_frame, "semantic_map", markers.markers);

        int i = 0;
        for (const auto& obstacle : semantic_map) {
            geometry_msgs::msg::Pose pose = obstacle.l_pose.pose;
            perception_utils::create_marker(pose.position.x, pose.position.y, pose.position.z, i++, map_frame,
                                            "semantic_map", perception_utils::Color::GREEN, obstacle.l_pose.label,
                                            markers.markers);
        }
        marker_pub->publish(markers);
    };
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticMapManager>());
    rclcpp::shutdown();
    return 0;
}
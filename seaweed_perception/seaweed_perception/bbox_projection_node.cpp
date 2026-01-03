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
#include "seaweed_perception/perception_utils.hpp"

// NOTE: assumes no distortion (ex. gazebo or zed 2i), add flag for distortion
class BBox_Projection_Node : public rclcpp::Node {
public:
    BBox_Projection_Node()
        : Node("bbox_projection_node"),
          rgb_image_topic("/wamv/sensors/cameras/camera_sensor/optical/image_raw"),
          depth_image_topic("/wamv/sensors/cameras/camera_sensor/optical/depth"),
          camera_info_topic("/wamv/sensors/cameras/camera_sensor/optical/camera_info"),
          detection_topic("/cv_detections"),
          base_link_frame("wamv/base_link"),
          map_frame("map"),
          camera_optical_frame("wamv/base_link/camera_sensor_optical"),
          recieved_img(false),
          intrinsics_set(false),
          image_expiration_threshold(3.0),
          sample_scaling_factor(.05),
          mad_threshold_multiplier(1.4826) {
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        detection_sub = this->create_subscription<seaweed_interfaces::msg::Detection>(
            detection_topic, 10,
            std::bind(&BBox_Projection_Node::detection_callback, this, std::placeholders::_1));
        rgb_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            rgb_image_topic, perception_utils::image_qos,
            std::bind(&BBox_Projection_Node::rgb_image_callback, this, std::placeholders::_1));
        depth_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            depth_image_topic, perception_utils::image_qos,
            std::bind(&BBox_Projection_Node::depth_image_callback, this, std::placeholders::_1));

        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10,
            std::bind(&BBox_Projection_Node::camera_info_callback, this, std::placeholders::_1));

        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/markers", 1);
    };

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Subscription<seaweed_interfaces::msg::Detection>::SharedPtr detection_sub;

    std::string rgb_image_topic;
    std::string depth_image_topic;
    std::string camera_info_topic;
    std::string detection_topic;

    std::string base_link_frame;
    std::string map_frame;
    std::string camera_optical_frame;

    cv::Mat latest_rgb_image;
    cv::Mat latest_depth_image;

    std::vector<seaweed_interfaces::msg::BoundingBox> detections;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

    rclcpp::Time camera_update_timestamp;

    cv_bridge::CvImagePtr cv_ptr;
    bool recieved_img;
    bool intrinsics_set;
    float image_expiration_threshold;

    perception_utils::CameraIntrinsics camera_intrinsics;

    float sample_scaling_factor;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    std::vector<perception_utils::Point> depth_sample_points;
    std::vector<float> depths;
    float mad_threshold_multiplier;

    void rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            latest_rgb_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            recieved_img = true;
            camera_update_timestamp = this->get_clock()->now();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge rgb error: %s", e.what());
            return;
        }
    };

    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            latest_depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            recieved_img = true;
            camera_update_timestamp = this->get_clock()->now();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge depth error: %s", e.what());
            return;
        }
    };

    bool is_image_valid(const rclcpp::Time timestamp, float expiration_seconds) {
        rclcpp::Duration time_since_last_update = this->get_clock()->now() - timestamp;
        return time_since_last_update.seconds() <= expiration_seconds;
    };

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (intrinsics_set) {
            return;
        }
        camera_intrinsics.set_from_cam_info(*msg);
        intrinsics_set = true;
    };

    void detection_callback(const seaweed_interfaces::msg::Detection::SharedPtr msg) {
        if (msg->detections.empty()) {
            RCLCPP_WARN(this->get_logger(), "got no detections");
            return;
        }
        if (!recieved_img || !intrinsics_set ||
            !is_image_valid(camera_update_timestamp, image_expiration_threshold)) {
            RCLCPP_WARN(this->get_logger(), "No valid image");
            return;
        }

        detections = msg->detections;

        if (detections.empty()) {
            RCLCPP_INFO(this->get_logger(), "No detections");
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "%zu detections", detections.size());

        geometry_msgs::msg::PoseArray proj_poses_c;
        geometry_msgs::msg::PoseArray proj_poses_w;

        proj_poses_c.header.frame_id = camera_optical_frame;
        proj_poses_c.header.stamp = msg->header.stamp;

        for (const seaweed_interfaces::msg::BoundingBox& bbox : detections) {
            int u = bbox.x + bbox.width / 2;
            int v = bbox.y + bbox.height / 2;

            float depth =
                sample_depth(depth_sample_points, depths, u, v, bbox.width, bbox.height, sample_scaling_factor);

            depth_sample_points.clear();
            depths.clear();

            RCLCPP_INFO(this->get_logger(), "depth: %f", depth);

            if (depth <= 0) {
                RCLCPP_WARN(this->get_logger(), "invalid bbox depth at (%d, %d), SKIPPING", u, v);
                continue;
            }

            auto [x_c, y_c, z_c] = camera_intrinsics.project_to_3d(u, v, depth);

            // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", x_c, y_c, z_c);

            geometry_msgs::msg::Pose pose_c;
            pose_c.position.x = x_c;
            pose_c.position.y = y_c;
            pose_c.position.z = z_c;
            pose_c.orientation.w = 1.0;
            proj_poses_c.poses.push_back(pose_c);
        }

        if (!perception_utils::transform_pose_array(proj_poses_c, proj_poses_w, base_link_frame, tf_buffer,
                                                    get_logger())) {
            RCLCPP_ERROR(this->get_logger(), "bbox transform pose array failed");
            return;
        }

        visualize_pose_array(proj_poses_w);
    };

    float get_depth_at_pixel(const int u, const int v) {
        float depth_meters = latest_depth_image.at<float>(v, u);
        if (!std::isfinite(depth_meters)) {
            // NOTE: test this w/ zed
            return -1;
        }
        return depth_meters;
    };

    float sample_depth(std::vector<perception_utils::Point>& samples, std::vector<float>& _depths, const int u,
                       const int v, const int bbox_width, const int bbox_height, const float scaling_factor) {
        int horizontal_offset = bbox_width * scaling_factor;
        int vertical_offset = bbox_height * scaling_factor;

        for (const auto& [pu, pv] : {std::pair{u, v},
                                     // plus
                                     {u + horizontal_offset, v},
                                     {u - horizontal_offset, v},
                                     {u, v + vertical_offset},
                                     {u, v - vertical_offset},
                                     // diagonal
                                     {u + horizontal_offset, v + vertical_offset},
                                     {u + horizontal_offset, v - vertical_offset},
                                     {u - horizontal_offset, v + vertical_offset},
                                     {u - horizontal_offset, v - vertical_offset}}) {
            float d = get_depth_at_pixel(pu, pv);
            if (d > 0) {
                samples.push_back(perception_utils::Point{(float)pu, (float)pv});
                _depths.push_back(d);
            }
        }
        // remove depth stat outliers
        float median = calc_median(_depths);
        float mad_threshold = calc_med_abs_dev_threshold(_depths, median, mad_threshold_multiplier);

        size_t write_idx = 0;
        for (size_t read_idx = 0; read_idx < _depths.size(); ++read_idx) {
            if (std::abs(_depths[read_idx] - median) <= mad_threshold) {
                _depths[write_idx] = _depths[read_idx];
                ++write_idx;
            }
        }
        _depths.resize(write_idx);

        // if no valid depths
        if (_depths.empty()) {
            return -1.0f;
        }

        // calc average of valid depths
        float sum = 0;
        for (auto depth : _depths) {
            sum += depth;
        }

        return sum / _depths.size();
    };

    float calc_median(std::vector<float>& values) {
        std::sort(values.begin(), values.end());
        int n = values.size();
        if (n % 2 == 0) {
            return (values[n / 2 - 1] + values[n / 2]) / 2.0f;
        }
        return values[n / 2];
    };

    float calc_med_abs_dev_threshold(std::vector<float> values, float median, float _threshold_multiplier) {
        for (auto& value : values) {
            value = std::abs(value - median);
        }
        float MAD = calc_median(values);
        return MAD * _threshold_multiplier;
    };

    void visualize_pose_array(geometry_msgs::msg::PoseArray pose_array) {
        visualization_msgs::msg::MarkerArray marker_array;
        std::string frame = pose_array.header.frame_id;
        perception_utils::reset_markers(frame, "bbox_projections_node", marker_array.markers);

        perception_utils::create_marker(1, 1, 0, -100, frame, "bbox_projections_node",
                                        perception_utils::Color::GREEN, "bbox_test", marker_array.markers);
        int i = 0;
        for (auto const& pose : pose_array.poses) {
            perception_utils::create_marker(pose.position.x, pose.position.y, pose.position.z, i, frame,
                                            "bbox_projections_node", perception_utils::Color::GREEN, "bbox",
                                            marker_array.markers);
            i++;
        }
        marker_pub->publish(marker_array);
    };
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBox_Projection_Node>());
    rclcpp::shutdown();
    return 0;
};

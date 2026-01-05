#include "seaweed_perception/bbox_projection_node.hpp"

BBox_Projection_Node::BBox_Projection_Node()
    : Node("bbox_projection_node"),
      rgb_image_topic("/wamv/sensors/cameras/camera_sensor/optical/image_raw"),
      depth_image_topic("/wamv/sensors/cameras/camera_sensor/optical/depth"),
      camera_info_topic("/wamv/sensors/cameras/camera_sensor/optical/camera_info"),
      detection_topic("/cv_detections"),
      projection_topic("/bbox_projection"),
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
        detection_topic, 10, std::bind(&BBox_Projection_Node::detection_callback, this, std::placeholders::_1));
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
    projection_pub = this->create_publisher<seaweed_interfaces::msg::LabeledPoseArray>(projection_topic, 10);

    bbox_projections.header.frame_id = map_frame;
};

// DONT USE SO PERHAPS REMOVE
void BBox_Projection_Node::rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        latest_rgb_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        recieved_img = true;
        camera_update_timestamp = this->get_clock()->now();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge rgb error: %s", e.what());
        return;
    }
};

void BBox_Projection_Node::depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        latest_depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        recieved_img = true;
        camera_update_timestamp = this->get_clock()->now();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge depth error: %s", e.what());
        return;
    }
};

void BBox_Projection_Node::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (intrinsics_set) {
        return;
    }
    camera_intrinsics.set_from_cam_info(*msg);
    intrinsics_set = true;
};

void BBox_Projection_Node::detection_callback(const seaweed_interfaces::msg::Detection::SharedPtr msg) {
    if (msg->detections.empty()) {
        RCLCPP_WARN(this->get_logger(), "got no detections");
        return;
    }
    if (!recieved_img || !intrinsics_set || !is_image_valid(camera_update_timestamp, image_expiration_threshold)) {
        RCLCPP_WARN(this->get_logger(), "No valid image");
        return;
    }

    detections = msg->detections;

    if (detections.empty()) {
        RCLCPP_INFO(this->get_logger(), "No detections");
        return;
    }

    seaweed_interfaces::msg::LabeledPoseArray proj_poses_c, proj_poses_w;

    proj_poses_c.header.frame_id = camera_optical_frame;
    proj_poses_c.header.stamp = msg->header.stamp;

    // proj_poses_w.header.frame_id = map_frame;
    proj_poses_w.header.frame_id = base_link_frame;
    proj_poses_w.header.stamp = msg->header.stamp;

    for (const seaweed_interfaces::msg::BoundingBox& bbox : detections) {
        int u = bbox.x + bbox.width / 2;
        int v = bbox.y + bbox.height / 2;

        float depth =
            sample_depth(depth_sample_points, depths, u, v, bbox.width, bbox.height, sample_scaling_factor);

        depth_sample_points.clear();
        depths.clear();

        if (depth <= 0) {
            RCLCPP_WARN(this->get_logger(), "invalid bbox depth at (%d, %d), SKIPPING", u, v);
            continue;
        }

        auto [x_c, y_c, z_c] = camera_intrinsics.project_to_3d(u, v, depth);

        // RCLCPP_INFO(this->get_logger(), "depth: %f", depth);
        // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", x_c, y_c, z_c);

        seaweed_interfaces::msg::LabeledPose pose_c;
        pose_c.pose.position.x = x_c;
        pose_c.pose.position.y = y_c;
        pose_c.pose.position.z = z_c;
        pose_c.pose.orientation.w = 1.0;
        pose_c.label = bbox.label;
        proj_poses_c.labeled_poses.push_back(pose_c);
    }

    if (!perception_utils::transform_labeled_pose_array(proj_poses_c, proj_poses_w, proj_poses_w.header.frame_id,
                                                        tf_buffer, get_logger())) {
        RCLCPP_ERROR(this->get_logger(), "bbox transform labeled pose array failed");
        return;
    }

    visualize_labeled_pose_array(proj_poses_w);
    projection_pub->publish(proj_poses_w);
};

bool BBox_Projection_Node::is_image_valid(const rclcpp::Time timestamp, float expiration_seconds) {
    rclcpp::Duration time_since_last_update = this->get_clock()->now() - timestamp;
    return time_since_last_update.seconds() <= expiration_seconds;
};

float BBox_Projection_Node::get_depth_at_pixel(const int u, const int v) {
    float depth_meters = latest_depth_image.at<float>(v, u);
    if (!std::isfinite(depth_meters)) {
        return -1;
    }
    return depth_meters;
};

float BBox_Projection_Node::sample_depth(std::vector<perception_utils::Point>& samples,
                                         std::vector<float>& _depths, const int u, const int v,
                                         const int bbox_width, const int bbox_height, const float scaling_factor) {
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
            samples.push_back(perception_utils::Point{(float)pu, (float)pv, 0.0f});
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

float BBox_Projection_Node::calc_median(std::vector<float>& values) {
    std::sort(values.begin(), values.end());
    int n = values.size();
    if (n % 2 == 0) {
        return (values[n / 2 - 1] + values[n / 2]) / 2.0f;
    }
    return values[n / 2];
};

float BBox_Projection_Node::calc_med_abs_dev_threshold(std::vector<float> values, float median,
                                                       float _threshold_multiplier) {
    for (auto& value : values) {
        value = std::abs(value - median);
    }
    float MAD = calc_median(values);
    return MAD * _threshold_multiplier;
};

void BBox_Projection_Node::visualize_labeled_pose_array(
    const seaweed_interfaces::msg::LabeledPoseArray& labaled_pose_array) {
    visualization_msgs::msg::MarkerArray marker_array;
    std::string frame = labaled_pose_array.header.frame_id;
    perception_utils::reset_markers(frame, "bbox_projections_node", marker_array.markers);

    int i = 0;
    for (auto const& labaled_pose : labaled_pose_array.labeled_poses) {
        perception_utils::create_marker(labaled_pose.pose.position.x, labaled_pose.pose.position.y,
                                        labaled_pose.pose.position.z, i, frame, "bbox_projections_node",
                                        perception_utils::Color::GREEN, labaled_pose.label, marker_array.markers);
        i++;
    }
    marker_pub->publish(marker_array);
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBox_Projection_Node>());
    rclcpp::shutdown();
    return 0;
};
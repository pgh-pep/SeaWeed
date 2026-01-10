#include "seaweed_perception/bbox_projection_node.hpp"

BBox_Projection_Node::BBox_Projection_Node()
    : Node("bbox_projection_node"),
      debug(false),
      depth_image_topic("/wamv/sensors/cameras/camera_sensor/optical/depth"),
      yolo_bbox_image_topic("/debug/yolo"),
      camera_info_topic("/wamv/sensors/cameras/camera_sensor/optical/camera_info"),
      detection_topic("/cv_detections"),
      cluster_topic("/clusters"),
      matched_topic("/mapping/matched"),
      unmatched_bbox_topic("/mapping/unmatched_bbox"),
      unmatched_cluster_topic("/mapping/unmatched_clusters"),
      base_link_frame("wamv/base_link"),
      map_frame("map"),
      camera_optical_frame("wamv/base_link/camera_sensor_optical"),
      recieved_yolo(false),
      recieved_depth(false),
      recieved_clusters(false),
      intrinsics_set(false),
      expiration_thresh(5.0),
      sample_scaling_factor(.05),
      mad_threshold_multiplier(1.4826),  // recc constant as per wikepedia
      max_assignment_cost(100.0),
      bbox_margin(0.1) {
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    detection_sub = this->create_subscription<seaweed_interfaces::msg::Detection>(
        detection_topic, 10, std::bind(&BBox_Projection_Node::detection_callback, this, std::placeholders::_1));

    cluster_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        cluster_topic, 10, std::bind(&BBox_Projection_Node::cluster_callback, this, std::placeholders::_1));

    depth_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        depth_image_topic, perception_utils::image_qos,
        std::bind(&BBox_Projection_Node::depth_image_callback, this, std::placeholders::_1));

    yolo_debug_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        yolo_bbox_image_topic, perception_utils::image_qos,
        std::bind(&BBox_Projection_Node::yolo_image_callback, this, std::placeholders::_1));

    debug_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/debug/cluster_bbox_projections",
                                                                      perception_utils::image_qos);

    camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&BBox_Projection_Node::camera_info_callback, this, std::placeholders::_1));

    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/markers", 1);
    matched_pub = this->create_publisher<seaweed_interfaces::msg::LabeledPoseArray>(matched_topic, 10);
    unmatched_bbox_pub =
        this->create_publisher<seaweed_interfaces::msg::LabeledPoseArray>(unmatched_bbox_topic, 10);
    unmatched_cluster_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(unmatched_cluster_topic, 10);
}

float BBox_Projection_Node::calc_cost(const seaweed_interfaces::msg::BoundingBox& bbox,
                                      const ClusterProjection& proj) {
    int u_min = bbox.x + bbox.width * bbox_margin;
    int u_max = bbox.x + bbox.width * (1.0 - bbox_margin);
    int v_min = bbox.y + bbox.height * bbox_margin;
    int v_max = bbox.y + bbox.height * (1.0 - bbox_margin);

    // current cost = depth if inside box (prioritize closer clusters), inf if outside
    // TODO: handle infinity better?
    if (proj.u < u_min || proj.u > u_max || proj.v < v_min || proj.v > v_max) {
        return 1e9f;
    }

    return proj.depth_c;
}

void BBox_Projection_Node::cluster_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    latest_clusters = *msg;
    cluster_update_stamp = this->get_clock()->now();
    recieved_clusters = true;
}

void BBox_Projection_Node::match_detections_to_clusters(
    const std::vector<seaweed_interfaces::msg::BoundingBox>& bboxes, std::vector<ClusterProjection>& projections,
    std::vector<std::pair<int, int>>& matches_idx, std::vector<int>& unmatched_bboxes_idx,
    std::vector<int>& unmatched_clusters_idx) {
    matches_idx.clear();
    unmatched_bboxes_idx.clear();
    unmatched_clusters_idx.clear();

    if (bboxes.empty()) {
        for (size_t j = 0; j < projections.size(); ++j) {
            unmatched_clusters_idx.push_back((int)j);
        }
        return;
    }

    if (projections.empty()) {
        for (size_t i = 0; i < bboxes.size(); ++i) {
            unmatched_bboxes_idx.push_back((int)i);
        }
        return;
    }

    int num_bboxes = bboxes.size();
    int num_clusters = projections.size();

    // cost matrix: rows = bboxes, cols = clusters
    std::vector<std::vector<float>> cost(num_bboxes, std::vector<float>(num_clusters));

    for (int i = 0; i < num_bboxes; ++i) {
        for (int j = 0; j < num_clusters; ++j) {
            cost[i][j] = calc_cost(bboxes[i], projections[j]);
        }
    }

    HungarianResult result = hungarian_solve(cost, max_assignment_cost);

    for (int i = 0; i < num_bboxes; ++i) {
        int j = result.row_to_col[i];
        if (j >= 0) {
            matches_idx.emplace_back(i, j);
            projections[j].matched = true;
        } else {
            unmatched_bboxes_idx.push_back(i);
        }
    }

    for (int j = 0; j < num_clusters; ++j) {
        if (result.col_to_row[j] < 0) {
            unmatched_clusters_idx.push_back(j);
        }
    }
}

void BBox_Projection_Node::depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        latest_depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        depth_update_stamp = this->get_clock()->now();
        recieved_depth = true;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge depth error: %s", e.what());
    }
}

void BBox_Projection_Node::yolo_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        latest_yolo_debug_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        recieved_yolo = true;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge yolo error: %s", e.what());
    }
}

void BBox_Projection_Node::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (intrinsics_set) return;
    camera_intrinsics.set_from_cam_info(*msg);
    intrinsics_set = true;
    RCLCPP_INFO(this->get_logger(), "set cam intrinsics");
}

void BBox_Projection_Node::detection_callback(const seaweed_interfaces::msg::Detection::SharedPtr msg) {
    if (!intrinsics_set) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "waiting for cam intrinsics");
        return;
    }

    bool clusters_valid = recieved_clusters && is_data_valid(cluster_update_stamp);
    bool depth_valid = recieved_depth && is_data_valid(depth_update_stamp);

    if (!clusters_valid || !depth_valid) {
        RCLCPP_WARN(this->get_logger(), "sensor data not valid");
        return;
    }

    // transform cluster pose onto camera projection
    std::vector<ClusterProjection> cluster_projections;
    clusters_to_projections(latest_clusters, cluster_projections);

    // pair: bbox idx -> cluster idx (might replace w/ a struct)
    std::vector<std::pair<int, int>> matches_idx;
    std::vector<int> unmatched_bboxes_idx;
    std::vector<int> unmatched_clusters_idx;

    match_detections_to_clusters(msg->detections, cluster_projections, matches_idx, unmatched_bboxes_idx,
                                 unmatched_clusters_idx);

    seaweed_interfaces::msg::LabeledPoseArray matched_poses;
    matched_poses.header.frame_id = latest_clusters.header.frame_id;
    matched_poses.header.stamp = msg->header.stamp;

    seaweed_interfaces::msg::LabeledPoseArray unmatched_bbox_poses;
    unmatched_bbox_poses.header.frame_id = camera_optical_frame;
    unmatched_bbox_poses.header.stamp = msg->header.stamp;

    geometry_msgs::msg::PoseArray unmatched_cluster_poses;
    unmatched_cluster_poses.header.frame_id = latest_clusters.header.frame_id;
    unmatched_cluster_poses.header.stamp = msg->header.stamp;

    // MATCHED BBOX TO CLUSTER
    // use cluster location w/ bbox label
    for (const auto& [bbox_idx, cluster_idx] : matches_idx) {
        const seaweed_interfaces::msg::BoundingBox& bbox = msg->detections.at(bbox_idx);
        const ClusterProjection& cluster = cluster_projections.at(cluster_idx);

        seaweed_interfaces::msg::LabeledPose l_pose;
        l_pose.label = bbox.label;
        l_pose.pose.position = cluster.pos;
        l_pose.pose.orientation.w = 1.0;
        matched_poses.labeled_poses.push_back(l_pose);

        RCLCPP_DEBUG(this->get_logger(), "matched bbox %s to cluster %d", bbox.label.c_str(), cluster_idx);
    }

    // UNMATCHED BBOXES
    for (int i : unmatched_bboxes_idx) {
        geometry_msgs::msg::Pose pose_c;
        if (bbox_to_pose_c(msg->detections[i], pose_c)) {
            // IF TOO FAR, FILTER OUT (m)
            if (pose_c.position.z > 10.0) {
                continue;
            }

            seaweed_interfaces::msg::LabeledPose labeled;
            labeled.label = msg->detections[i].label;

            labeled.pose = pose_c;
            unmatched_bbox_poses.labeled_poses.push_back(labeled);

            RCLCPP_DEBUG(this->get_logger(), "unmatched bbox: %s", msg->detections[i].label.c_str());
        }
    }

    // UNMATCHED CLUSTERS
    for (int j : unmatched_clusters_idx) {
        geometry_msgs::msg::Pose pose;
        pose.position = cluster_projections[j].pos;
        pose.orientation.w = 1.0;
        unmatched_cluster_poses.poses.push_back(pose);
    }

    // map frame transforms
    seaweed_interfaces::msg::LabeledPoseArray matched_poses_map;
    matched_poses_map.header.frame_id = map_frame;
    matched_poses_map.header.stamp = msg->header.stamp;

    seaweed_interfaces::msg::LabeledPoseArray unmatched_bboxes_map;
    unmatched_bboxes_map.header.frame_id = map_frame;
    unmatched_bboxes_map.header.stamp = msg->header.stamp;

    geometry_msgs::msg::PoseArray unmatched_clusters_map;
    unmatched_clusters_map.header.frame_id = map_frame;
    unmatched_clusters_map.header.stamp = msg->header.stamp;

    if (!matched_poses.labeled_poses.empty()) {
        if (!perception_utils::transform_labeled_pose_array(matched_poses, matched_poses_map, map_frame, tf_buffer,
                                                            this->get_logger())) {
            RCLCPP_ERROR(this->get_logger(), "matched pose transform failure");
            return;
        }
    }

    if (!unmatched_bbox_poses.labeled_poses.empty()) {
        if (!perception_utils::transform_labeled_pose_array(unmatched_bbox_poses, unmatched_bboxes_map, map_frame,
                                                            tf_buffer, this->get_logger())) {
            RCLCPP_ERROR(this->get_logger(), "unmatched bbox transform failure");
            return;
        }
    }

    if (!unmatched_cluster_poses.poses.empty()) {
        if (!perception_utils::transform_pose_array(unmatched_cluster_poses, unmatched_clusters_map, map_frame,
                                                    tf_buffer, this->get_logger())) {
            RCLCPP_ERROR(this->get_logger(), "unmatched cluster transform failure");
            return;
        }
    }

    matched_pub->publish(matched_poses_map);
    unmatched_bbox_pub->publish(unmatched_bboxes_map);
    unmatched_cluster_pub->publish(unmatched_clusters_map);

    if (debug) {
        visualize_on_img(cluster_projections);
        debug_markers(matched_poses_map, unmatched_bboxes_map, unmatched_clusters_map);

        RCLCPP_DEBUG(this->get_logger(), "matched: %lu, unmatched bbox: %lu, unmatched cluster : %lu",
                     matched_poses_map.labeled_poses.size(), unmatched_bboxes_map.labeled_poses.size(),
                     unmatched_clusters_map.poses.size());
    }
}

void BBox_Projection_Node::clusters_to_projections(const geometry_msgs::msg::PoseArray& clusters,
                                                   std::vector<ClusterProjection>& cluster_projections) {
    geometry_msgs::msg::PoseArray clusters_c;
    clusters_c.header.frame_id = camera_optical_frame;
    perception_utils::transform_pose_array(clusters, clusters_c, camera_optical_frame, tf_buffer,
                                           this->get_logger());

    for (size_t i = 0; i < clusters_c.poses.size(); i++) {
        const auto& cluster_c = clusters_c.poses[i];

        auto [u, v] = camera_intrinsics.projection_to_pixel(cluster_c.position.x, cluster_c.position.y,
                                                            cluster_c.position.z);
        ClusterProjection proj;
        proj.u = u;
        proj.v = v;
        proj.depth_c = cluster_c.position.z;
        proj.pos = clusters.poses[i].position;  // in original cluster frame (believe it is base_link)
        proj.matched = false;

        cluster_projections.push_back(proj);
    }
}

int BBox_Projection_Node::find_cluster_in_bbox(const seaweed_interfaces::msg::BoundingBox& bbox,
                                               std::vector<ClusterProjection>& projections) {
    int u_min = bbox.x + bbox.width * bbox_margin;
    int u_max = bbox.x + bbox.width * (1.0 - bbox_margin);
    int v_min = bbox.y + bbox.height * bbox_margin;
    int v_max = bbox.y + bbox.height * (1.0 - bbox_margin);

    int best_index = -1;
    float best_depth = std::numeric_limits<float>::max();

    for (size_t i = 0; i < projections.size(); ++i) {
        auto& proj = projections[i];

        if (proj.matched) continue;  // projection alr assigned to a bbox

        if (proj.u >= u_min && proj.u <= u_max && proj.v >= v_min && proj.v <= v_max) {
            // NOTE: currently prioritizes closest cluster
            if (proj.depth_c < best_depth) {
                best_depth = proj.depth_c;
                best_index = i;
            }
        }
    }

    return best_index;
}

bool BBox_Projection_Node::bbox_to_pose_c(const seaweed_interfaces::msg::BoundingBox& bbox,
                                          geometry_msgs::msg::Pose& pose_c) {
    int u = bbox.x + bbox.width / 2;
    int v = bbox.y + bbox.height / 2;

    float depth = sample_depth(u, v, bbox.width, bbox.height);

    if (depth <= 0) {
        return false;
    }

    auto [x_c, y_c, z_c] = camera_intrinsics.pixel_to_projection(u, v, depth);

    pose_c.position.x = x_c;
    pose_c.position.y = y_c;
    pose_c.position.z = z_c;
    pose_c.orientation.w = 1.0;

    return true;
}

float BBox_Projection_Node::sample_depth(int u, int v, int bbox_width, int bbox_height) {
    std::vector<float> depths;

    int horiz_offset = bbox_width * sample_scaling_factor;
    int vert_offset = bbox_height * sample_scaling_factor;

    // currently samples plus and diagonal shape, will need to test if good idea
    for (const auto& [pu, pv] : {std::pair{u, v},
                                 {u + horiz_offset, v},
                                 {u - horiz_offset, v},
                                 {u, v + vert_offset},
                                 {u, v - vert_offset},
                                 {u + horiz_offset, v + vert_offset},
                                 {u + horiz_offset, v - vert_offset},
                                 {u - horiz_offset, v + vert_offset},
                                 {u - horiz_offset, v - vert_offset}}) {
        float d = get_depth_at_pixel(pu, pv);
        if (d > 0) {
            depths.push_back(d);
        }
    }

    if (depths.empty()) return -1.0f;

    float median = calc_median(depths);
    float mad_threshold = calc_mad_threshold(depths, median);

    // calc average of valid depths (remove w/ mad threshold)
    float sum = 0;
    int count = 0;
    for (float depth : depths) {
        if (std::abs(depth - median) <= mad_threshold) {
            sum += depth;
            count++;
        }
    }

    // if filter out all depths, return -1 (shouldn't ever happen though)
    return count > 0 ? sum / count : -1.0f;
}

float BBox_Projection_Node::get_depth_at_pixel(int u, int v) {
    if (u < 0 || u >= latest_depth_image.cols || v < 0 || v >= latest_depth_image.rows) {
        return -1;
    }
    float depth = latest_depth_image.at<float>(v, u);
    // TODO: make sure this works w/ zed
    return std::isfinite(depth) ? depth : -1;
}

float BBox_Projection_Node::calc_median(std::vector<float>& values) {
    if (values.empty()) {
        return 0;
    }

    std::sort(values.begin(), values.end());
    size_t n = values.size();
    if ((int)n % 2 == 0) {
        return (values[n / 2 - 1] + values[n / 2]) / 2.0f;
    }
    return values[n / 2];
}

float BBox_Projection_Node::calc_mad_threshold(std::vector<float> values, float median) {
    for (auto& v : values) {
        v = std::abs(v - median);
    }
    float MAD = calc_median(values);
    return MAD * mad_threshold_multiplier;
}

bool BBox_Projection_Node::is_data_valid(const rclcpp::Time& stamp) {
    return (this->get_clock()->now() - stamp).seconds() <= expiration_thresh;
}

void BBox_Projection_Node::debug_markers(const seaweed_interfaces::msg::LabeledPoseArray& matched_poses,
                                         const seaweed_interfaces::msg::LabeledPoseArray& unmatched_bbox_poses,
                                         const geometry_msgs::msg::PoseArray& unmatched_cluster_poses) {
    visualization_msgs::msg::MarkerArray markers;
    perception_utils::reset_markers(map_frame, "bbox_projection", markers.markers);

    int i = 0;

    for (const auto& pose : matched_poses.labeled_poses) {
        perception_utils::create_marker(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, i++,
                                        map_frame, "bbox_projection", perception_utils::Color::GREEN, pose.label,
                                        markers.markers);
    }

    for (const auto& pose : unmatched_bbox_poses.labeled_poses) {
        perception_utils::create_marker(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, i++,
                                        map_frame, "bbox_projection", perception_utils::Color::RED, pose.label,
                                        markers.markers);
    }

    for (const auto& pose : unmatched_cluster_poses.poses) {
        perception_utils::create_marker(pose.position.x, pose.position.y, pose.position.z, i++, map_frame,
                                        "bbox_projection", perception_utils::Color::PURPLE, "UNMATCHED CLUSTER",
                                        markers.markers);
    }

    marker_pub->publish(markers);
}

void BBox_Projection_Node::visualize_on_img(const std::vector<ClusterProjection>& projections) {
    if (!recieved_yolo || latest_yolo_debug_image.empty()) {
        return;
    }

    cv::Mat debug_img = latest_yolo_debug_image.clone();

    for (const auto& proj : projections) {
        cv::Scalar color = proj.matched ? cv::Scalar(0, 255, 0)   // green = matched
                                        : cv::Scalar(0, 0, 255);  // red = unmatched

        cv::circle(debug_img, cv::Point(proj.u, proj.v), 6, color, cv::FILLED);

        std::stringstream depth_ss;
        depth_ss << std::fixed << std::setprecision(1) << proj.depth_c;
        // note: may be an issue if bbox is on right of screen:
        cv::putText(debug_img, depth_ss.str(), cv::Point(proj.u + 10, proj.v), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    color, 1);
    }

    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_img).toImageMsg();
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = camera_optical_frame;
    debug_image_pub->publish(*msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBox_Projection_Node>());
    rclcpp::shutdown();
    return 0;
}
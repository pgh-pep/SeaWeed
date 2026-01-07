#include "seaweed_perception/euclidian_clustering_node.hpp"

EuclidianClusteringNode::EuclidianClusteringNode()
    : Node("euclidian_clustering_node"),
      box_range(10.0),
      leaf_size(0.01),
      clustering_tolerance(0.5),
      scale(.3),
      min_cluster_points(5),
      cluster_topic("/clusters"),
      base_link("wamv/base_link") {
    pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10,
        std::bind(&EuclidianClusteringNode::pc_callback, this, std::placeholders::_1));

    debug_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/pointcloud", 10);
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/markers", 1);

    cluster_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(cluster_topic, 10);

    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    RCLCPP_INFO(this->get_logger(), "started euclidian clustering node");
}

void EuclidianClusteringNode::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pc_with_boat(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pc_with_outliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pc(new pcl::PointCloud<pcl::PointXYZ>);

    // INPUT
    perception_utils::ros_to_pcl(msg, pc);
    perception_utils::transform_pc(pc, transformed_pc, base_link, tf_buffer, this->get_logger());

    // FILTERING
    filter_box(transformed_pc, cropped_pc, box_range);
    downsample(cropped_pc, downsampled_pc, leaf_size);
    plane_RANSAC(downsampled_pc, obstacle_pc_with_boat);
    filter_boat(obstacle_pc_with_boat, obstacle_pc_with_outliers);
    filter_outliers(obstacle_pc_with_outliers, obstacle_pc);

    // CLUSTERING
    scaled_euclidian_clustering(obstacle_pc, clusters, clustering_tolerance, scale, min_cluster_points);
    publish_clusters(clusters);

    // DEBUG OUTPUT
    perception_utils::debug_pointcloud(obstacle_pc, base_link, debug_pointcloud_pub, this->get_clock(),
                                       this->get_logger());
}

void EuclidianClusteringNode::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc, float leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(unfiltered_pc);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*filtered_pc);
}

void EuclidianClusteringNode::plane_RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.1);

    seg.setInputCloud(unfiltered_pc);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        RCLCPP_WARN(this->get_logger(), "failed to find indices from pc");
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(unfiltered_pc);
    extract.setIndices(inliers);
    extract.setNegative(false);

    extract.filter(*cloud_plane);
    RCLCPP_DEBUG(this->get_logger(), "pc representing the water plane: '%lu' data points.",
                 cloud_plane->points.size());

    extract.setNegative(true);
    extract.filter(*filtered_pc);
}

void EuclidianClusteringNode::filter_box(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc, float box_range) {
    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(unfiltered_pc);

    float height = 7.5;

    Eigen::Vector4f min_point = Eigen::Vector4f(-box_range, -box_range, 0, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(box_range, box_range, height, 0);
    crop_box_filter.setMin(min_point);
    crop_box_filter.setMax(max_point);
    crop_box_filter.filter(*filtered_pc);
}

void EuclidianClusteringNode::filter_boat(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc) {
    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(unfiltered_pc);

    float range = 3.0;
    float height = 5.0;

    Eigen::Vector4f min_point = Eigen::Vector4f(-range, -range, 0, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(range, range, height, 0);
    crop_box_filter.setMin(min_point);
    crop_box_filter.setMax(max_point);
    crop_box_filter.setNegative(true);
    crop_box_filter.filter(*filtered_pc);
}

void EuclidianClusteringNode::filter_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat_outlier_filter;
    stat_outlier_filter.setInputCloud(unfiltered_pc);
    stat_outlier_filter.setMeanK(50);
    stat_outlier_filter.setStddevMulThresh(1.0);
    stat_outlier_filter.filter(*filtered_pc);
}

void EuclidianClusteringNode::euclidian_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                                                   std::vector<perception_utils::Point>& _clusters,
                                                   float _clustering_tolerance, int _min_clustering_points) {
    _clusters.clear();

    if (pc->empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Empty point cloud, can't cluster");
        return;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pc);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> eucl_clustering_extraction;
    eucl_clustering_extraction.setClusterTolerance(_clustering_tolerance);
    eucl_clustering_extraction.setMinClusterSize(_min_clustering_points);
    eucl_clustering_extraction.setMaxClusterSize(10000);
    eucl_clustering_extraction.setSearchMethod(tree);
    eucl_clustering_extraction.setInputCloud(pc);
    eucl_clustering_extraction.extract(cluster_indices);

    visualization_msgs::msg::MarkerArray marker_array;
    perception_utils::reset_markers(base_link, "euclidian_clustering", marker_array.markers);

    int i = 0;
    for (const auto& cluster : cluster_indices) {
        float centroid_x = 0, centroid_y = 0, centroid_z = 0;
        for (const auto& idx : cluster.indices) {
            centroid_x += (*pc)[idx].x;
            centroid_y += (*pc)[idx].y;
            centroid_z += (*pc)[idx].z;
        }
        centroid_x /= cluster.indices.size();
        centroid_y /= cluster.indices.size();
        centroid_z /= cluster.indices.size();

        perception_utils::create_marker(centroid_x, centroid_y, centroid_z, i, base_link, "euclidian_clustering",
                                        perception_utils::Color::RED, "C" + std::to_string(i),
                                        marker_array.markers);

        _clusters.push_back({centroid_x, centroid_y, centroid_z});
        i++;
    }

    marker_pub->publish(marker_array);
    RCLCPP_DEBUG(this->get_logger(), "Found %d clusters", i);
}

void EuclidianClusteringNode::scaled_euclidian_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                                                          std::vector<perception_utils::Point>& _clusters,
                                                          float _clustering_tolerance, float _scale,
                                                          int _min_clustering_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scaled_pc(new pcl::PointCloud<pcl::PointXYZ>);
    *scaled_pc = *pc;

    for (auto& point : scaled_pc->points) {
        point.z *= _scale;
    }

    euclidian_clustering(scaled_pc, _clusters, _clustering_tolerance, _min_clustering_points);
}

void EuclidianClusteringNode::publish_clusters(const std::vector<perception_utils::Point>& clusters) {
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = base_link;
    msg.header.stamp = this->get_clock()->now();

    for (const auto& point : clusters) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = point.z;
        pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
    }

    cluster_pub->publish(msg);
}
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EuclidianClusteringNode>());
    rclcpp::shutdown();
    return 0;
}
#include "seaweed_mapping/euclidian_clustering_node.hpp"

#include "seaweed_mapping/mapping_utils.hpp"

EuclidianClusteringNode::EuclidianClusteringNode()
    : Node("euclidian_clustering_node"),
      box_range(10.0),
      leaf_size(0.01),
      clustering_tolerance(0.5),
      min_cluster_points(5),
      base_link("wamv/base_link") {
    pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10,
        std::bind(&EuclidianClusteringNode::pc_callback, this, std::placeholders::_1));

    debug_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/pointcloud", 10);
    marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/pointcloud_markers", 1);

    cluster_topic = "/debug/clusters";
    cluster_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(cluster_topic, 10);

    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    clusters = new std::vector<Point>;

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
    mapping_utils::ros_to_pcl(msg, pc);
    mapping_utils::transform_pc(pc, transformed_pc, base_link, tf_buffer, this->get_logger());

    // FILTERING
    filter_box(transformed_pc, cropped_pc, box_range);
    downsample(cropped_pc, downsampled_pc, leaf_size);
    remove_water_plane(downsampled_pc, obstacle_pc_with_boat);
    filter_boat(obstacle_pc_with_boat, obstacle_pc_with_outliers);
    filter_outliers(obstacle_pc_with_outliers, obstacle_pc);

    // CLUSTERING
    scaled_euclidian_clustering(obstacle_pc, clustering_tolerance, min_cluster_points, clusters);
    publish_clusters(clusters, cluster_topic, cluster_pub, this->get_clock());

    // DEBUG OUTPUT
    mapping_utils::debug_pointcloud(obstacle_pc, base_link, debug_pointcloud_pub, this->get_clock(),
                                    this->get_logger());
}

void EuclidianClusteringNode::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc, float leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(unfiltered_pc);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*filtered_pc);
}

void EuclidianClusteringNode::remove_water_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
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
                                                   float _clustering_tolerance, int _min_clustering_points,
                                                   std::vector<Point>* _clusters) {
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

    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> pc2_clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    Point point;

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

        RCLCPP_INFO(this->get_logger(), "cluster  #%i: x=%.2f, y=%.2f, z=%.2f, points=%lu", i, centroid_x,
                    centroid_y, centroid_z, cluster.indices.size());
        mapping_utils::create_marker(centroid_x, centroid_y, centroid_z, i, base_link, marker_pub, "bro");
        point.x = centroid_x;
        point.y = centroid_y;
        _clusters->push_back(point);
        i++;
    }
    // mapping_utils::reset_markers(base_link, marker_pub);
    RCLCPP_INFO(this->get_logger(), "Found '%i' clusters", i);
}

void EuclidianClusteringNode::scaled_euclidian_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                                                          float _clustering_tolerance, int _min_clustering_points,
                                                          std::vector<Point>* _clusters) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scaled_pc(new pcl::PointCloud<pcl::PointXYZ>);
    *scaled_pc = *pc;

    for (auto& point : scaled_pc->points) {
        point.z *= 0.3;
    }

    euclidian_clustering(scaled_pc, _clustering_tolerance, _min_clustering_points, _clusters);
}

void EuclidianClusteringNode::publish_clusters(
    std::vector<Point>* _clusters, std::string cluster_frame,
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher, rclcpp::Clock::SharedPtr clock) {
    geometry_msgs::msg::PoseArray::SharedPtr msg = std::make_shared<geometry_msgs::msg::PoseArray>();

    msg->header.frame_id = cluster_frame;
    msg->header.stamp = clock->now();

    for (const Point& point : *_clusters) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = 0.0;

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        msg->poses.push_back(pose);
    }

    publisher->publish(*msg);
    _clusters->clear();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EuclidianClusteringNode>());
    rclcpp::shutdown();
    return 0;
}
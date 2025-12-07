#pragma once

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "rclcpp/rclcpp.hpp"

class EuclidianClusteringNode : public rclcpp::Node {
public:
    EuclidianClusteringNode();

private:
    // SUB PUB
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // params
    float box_range;
    float leaf_size;
    float clustering_tolerance;
    int min_cluster_points;

    const std::string base_link;

    // Callbacks
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // FILTERS
    void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc, float leaf_size);

    void remove_water_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc);

    void filter_box(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc, float box_range);

    void filter_boat(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc);

    void filter_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc);

    // CLUSTERING
    void euclidian_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, float clustering_tolerance,
                              int min_clustering_points);

    void scaled_euclidian_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, float clustering_tolerance,
                                     int min_clustering_points);
};
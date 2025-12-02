#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.hpp>

#include "rclcpp/rclcpp.hpp"

// PCL Dependencies
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

#include <pcl_ros/transforms.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

class TestNode : public rclcpp::Node {
public:
    TestNode() : Node("test_node") {
        pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10,
            std::bind(&TestNode::pc_callback, this, std::placeholders::_1));

        debug_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/pointcloud", 10);

        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        RCLCPP_INFO(this->get_logger(), "started test node");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_pub;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    const std::string base_link = "wamv/base_link";

    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pc_unboxed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pc_with_boat(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pc_with_outliers(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pc(new pcl::PointCloud<pcl::PointXYZ>);

        // INPUT
        ros_to_pcl(msg, pc);
        transform_pc(pc, transformed_pc, base_link);

        // FILTERING
        downsample(transformed_pc, downsampled_pc);
        remove_water_plane(downsampled_pc, obstacle_pc_unboxed);
        filter_box(obstacle_pc_unboxed, obstacle_pc_with_boat);
        filter_boat(obstacle_pc_with_boat, obstacle_pc_with_outliers);
        filter_outliers(obstacle_pc_with_outliers, obstacle_pc);

        // DEBUG OUTPUT
        debug_pointcloud(obstacle_pc, base_link);
    }

    void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        float voxel = .1;
        voxel_filter.setInputCloud(unfiltered_pc);
        voxel_filter.setLeafSize(voxel, voxel, voxel);
        voxel_filter.filter(*filtered_pc);
    }

    void remove_water_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
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

    void filter_box(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc) {
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(unfiltered_pc);

        float range = 100.0;
        float height = 20.0;

        Eigen::Vector4f min_point = Eigen::Vector4f(-range, -range, 0, 0);
        Eigen::Vector4f max_point = Eigen::Vector4f(range, range, height, 0);
        crop_box_filter.setMin(min_point);
        crop_box_filter.setMax(max_point);
        crop_box_filter.filter(*filtered_pc);
    }

    void filter_boat(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
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

    void filter_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat_outlier_filter;
        stat_outlier_filter.setInputCloud(unfiltered_pc);
        stat_outlier_filter.setMeanK(50);
        stat_outlier_filter.setStddevMulThresh(1.0);
        stat_outlier_filter.filter(*filtered_pc);
    }

    void ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& pc_msg,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl) {
        pcl::fromROSMsg(*pc_msg, *pc_pcl);
    }

    void pcl_to_ros(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl,
                    sensor_msgs::msg::PointCloud2::SharedPtr& pc_msg, const std::string target_frame) {
        pcl::toROSMsg(*pc_pcl, *pc_msg);
        pc_msg->header.stamp = this->get_clock()->now();
        pc_msg->header.frame_id = target_frame;
    }

    void transform_pc(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_pc,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_pc, const std::string& target_frame) {
        try {
            geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer->lookupTransform(
                target_frame, original_pc->header.frame_id, rclcpp::Time{}, rclcpp::Duration::from_seconds(1.0));

            pcl_ros::transformPointCloud(*original_pc, *transformed_pc, tf_stamped);
            transformed_pc->header.frame_id = target_frame;

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "FAILED PC TRANSFORM: %s", ex.what());
        }
    }

    void debug_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, const std::string& target_frame) {
        sensor_msgs::msg::PointCloud2::SharedPtr debug_pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl_to_ros(pc, debug_pc_msg, target_frame);
        debug_pc_msg->header.frame_id = target_frame;

        debug_pointcloud_pub->publish(*debug_pc_msg);

        RCLCPP_INFO(this->get_logger(), "pub debug pc w/ %zu points", pc->size());
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}

// void remove_water_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc,
//                         pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_z;
//     pcl::PassThrough<pcl::PointXYZ> pass_through_filter_z;
//     pass_through_filter_z.setInputCloud(unfiltered_pc);
//     pass_through_filter_z.setFilterFieldName("z");
//     pass_through_filter_z.setNegative(true);
//     pass_through_filter_z.setFilterLimits(0.00, 0.5);
//     pass_through_filter_z.filter(*filtered_pc_z);

//     pcl::PassThrough<pcl::PointXYZ> pass_through_filter_z;
//     pass_through_filter_z.setInputCloud(filtered_pc_z);
//     pass_through_filter_z.setFilterFieldName("z");
//     pass_through_filter_z.setNegative(true);
//     pass_through_filter_z.setFilterLimits(0.00, 0.5);
//     pass_through_filter_z.filter(*filtered_pc);
// }
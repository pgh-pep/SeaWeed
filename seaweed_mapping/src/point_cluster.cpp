#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>

class PointCluster : public rclcpp::Node {
    public:
        PointCluster() : rclcpp::Node("PointClusterNode") {
            declare_parameter("pc_sub_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points");
            declare_parameter("pc_pub_topic", "/cluster/pointcloud");
            declare_parameter("pose_pub_topic", "/cluster/poses");
            declare_parameter("target_frame", "odom");
            declare_parameter("min_z", 0.0);
            declare_parameter("max_radius", 50.0);
            declare_parameter("robot_radius", 1.0);
            declare_parameter("pose_merge_distance", 0.1);
            declare_parameter("cluster_tolerance", 0.5);
            declare_parameter("min_cluster_size", 3);
            declare_parameter("max_cluster_size", 10000);

            pc_sub_topic_ = get_parameter("pc_sub_topic").as_string();
            pc_pub_topic_ = get_parameter("pc_pub_topic").as_string();
            pose_pub_topic_ = get_parameter("pose_pub_topic").as_string();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

            pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(pc_sub_topic_, 10, std::bind(&PointCluster::pc_callback, this, std::placeholders::_1));
            pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pc_pub_topic_, 10);
            pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(pose_pub_topic_, 10);
            pub_timer_ = create_timer(std::chrono::milliseconds(100), std::bind(&PointCluster::pub_timer_callback, this));
        }
    
    private:
        void pc_callback(const sensor_msgs::msg::PointCloud2& msg) {
            pc_in_ = msg;
            new_cloud_ = true;
        }

        void pub_timer_callback() {
            if (!new_cloud_) return;

            // Read each tick so params can be tuned at runtime
            const std::string target_frame = get_parameter("target_frame").as_string();
            const double min_z = get_parameter("min_z").as_double();
            const double max_radius = get_parameter("max_radius").as_double();
            const double robot_radius = get_parameter("robot_radius").as_double();
            const double pose_merge_distance = get_parameter("pose_merge_distance").as_double();
            const double cluster_tolerance = get_parameter("cluster_tolerance").as_double();
            const int min_cluster_size = get_parameter("min_cluster_size").as_int();
            const int max_cluster_size = get_parameter("max_cluster_size").as_int();

            // Transform into the target frame. On failure keep the cloud and retry next tick.
            geometry_msgs::msg::TransformStamped tf;
            sensor_msgs::msg::PointCloud2 pc_tf;
            try {
                tf = tf_buffer_->lookupTransform(target_frame, pc_in_.header.frame_id, pc_in_.header.stamp);
                tf2::doTransform(pc_in_, pc_tf, tf);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Could not transform cloud: %s", ex.what());
                return;
            }
            new_cloud_ = false;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(pc_tf, *cloud);

            // Radius filter around the sensor origin (expressed in the target frame)
            const double sx = tf.transform.translation.x;
            const double sy = tf.transform.translation.y;
            const double sz = tf.transform.translation.z;
            const double max_r2 = max_radius * max_radius;
            pcl::PointCloud<pcl::PointXYZ>::Ptr in_range(new pcl::PointCloud<pcl::PointXYZ>);
            in_range->reserve(cloud->size());
            for (const auto & pt : *cloud) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                const double dx = pt.x - sx, dy = pt.y - sy, dz = pt.z - sz;
                if (dx * dx + dy * dy + dz * dz <= max_r2) in_range->push_back(pt);
            }

            // 1. Minimum Z filter (in the target frame)
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(in_range);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_z, std::numeric_limits<float>::max());
            pass.filter(*filtered);

            // 2. Euclidean clustering, keeping only clustered points
            pcl::PointCloud<pcl::PointXYZ> clustered;
            poses_out_.poses.clear();

            if (!filtered->empty()) {
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(filtered);

                std::vector<pcl::PointIndices> clusters;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance(cluster_tolerance);
                ec.setMinClusterSize(min_cluster_size);
                ec.setMaxClusterSize(max_cluster_size);
                ec.setSearchMethod(tree);
                ec.setInputCloud(filtered);
                ec.extract(clusters);

                for (const auto & cluster : clusters) {
                    double sum_x = 0.0, sum_y = 0.0;
                    for (const int idx : cluster.indices) {
                        const auto & pt = (*filtered)[idx];
                        sum_x += pt.x;
                        sum_y += pt.y;
                    }
                    const double cx = sum_x / cluster.indices.size();
                    const double cy = sum_y / cluster.indices.size();

                    // Drop clusters on or near the robot itself
                    if (std::hypot(cx - sx, cy - sy) <= robot_radius) continue;

                    for (const int idx : cluster.indices) {
                        clustered.push_back((*filtered)[idx]);
                    }

                    // Skip poses too close to one that is already in the array
                    const bool duplicate = std::any_of(
                        poses_out_.poses.begin(), poses_out_.poses.end(),
                        [&](const geometry_msgs::msg::Pose & p) {
                            return std::hypot(cx - p.position.x, cy - p.position.y) < pose_merge_distance;
                        });
                    if (duplicate) continue;

                    // 3. Center of mass projected onto the XY plane (z = 0)
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = cx;
                    pose.position.y = cy;
                    pose.position.z = 0.0;
                    pose.orientation.w = 1.0;
                    poses_out_.poses.push_back(pose);
                }
            }

            pcl::toROSMsg(clustered, pc_out_);
            pc_out_.header = pc_tf.header;
            poses_out_.header = pc_tf.header;

            pc_pub_->publish(pc_out_);
            pose_pub_->publish(poses_out_);
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
        rclcpp::TimerBase::SharedPtr pub_timer_;
        
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        bool new_cloud_ = false;
        sensor_msgs::msg::PointCloud2 pc_in_;
        sensor_msgs::msg::PointCloud2 pc_out_;
        geometry_msgs::msg::PoseArray poses_out_;

        std::string pc_sub_topic_;
        std::string pose_pub_topic_;
        std::string pc_pub_topic_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCluster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
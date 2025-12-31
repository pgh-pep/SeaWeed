#include "seaweed_perception/perception_utils.hpp"

#include <tf2/exceptions.h>

#include <pcl_ros/transforms.hpp>

namespace perception_utils {

std::ostream& operator<<(std::ostream& os, const Point& point) {
    return os << "(" << point.x << ", " << point.y << ")";
}

std_msgs::msg::ColorRGBA get_rgba_color(Color color, float alpha) {
    std_msgs::msg::ColorRGBA rgba;
    rgba.a = alpha;

    switch (color) {
        case RED:
            rgba.r = 1.0f;
            rgba.g = 0.0f;
            rgba.b = 0.0f;
            break;
        case GREEN:
            rgba.r = 0.0f;
            rgba.g = 1.0f;
            rgba.b = 0.0f;
            break;
        case BLUE:
            rgba.r = 0.0f;
            rgba.g = 0.0f;
            rgba.b = 1.0f;
            break;
        default:
            rgba.r = 1.0f;
            rgba.g = 1.0f;
            rgba.b = 1.0f;
            break;
    }

    return rgba;
}

void ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& pc_msg,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl) {
    pcl::fromROSMsg(*pc_msg, *pc_pcl);
}

void pcl_to_ros(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl,
                sensor_msgs::msg::PointCloud2::SharedPtr& pc_msg, const std::string& target_frame,
                rclcpp::Clock::SharedPtr clock) {
    pcl::toROSMsg(*pc_pcl, *pc_msg);
    pc_msg->header.stamp = clock->now();
    pc_msg->header.frame_id = target_frame;
}

void transform_pc(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_pc,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_pc, const std::string& target_frame,
                  std::shared_ptr<tf2_ros::Buffer> tf_buffer, rclcpp::Logger logger) {
    try {
        geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer->lookupTransform(
            target_frame, original_pc->header.frame_id, rclcpp::Time{}, rclcpp::Duration::from_seconds(1.0));

        pcl_ros::transformPointCloud(*original_pc, *transformed_pc, tf_stamped);
        transformed_pc->header.frame_id = target_frame;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(logger, "FAILED PC TRANSFORM: %s", ex.what());
    }
}

void debug_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, const std::string& target_frame,
                      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                      rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger) {
    sensor_msgs::msg::PointCloud2::SharedPtr debug_pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pcl_to_ros(pc, debug_pc_msg, target_frame, clock);
    debug_pc_msg->header.frame_id = target_frame;
    debug_pc_msg->header.stamp = clock->now();

    publisher->publish(*debug_pc_msg);

    RCLCPP_DEBUG(logger, "pub debug pc w/ %zu points", pc->size());
}

void create_marker(const float& x, const float& y, const float& z, const int& id, const std::string& frame,
                   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher, const Color& color,
                   const std::string& label) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = rclcpp::Clock().now();

    marker.ns = "seaweed_perception_labels";
    marker.id = id;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    std_msgs::msg::ColorRGBA rgba = get_rgba_color(color);
    marker.color = rgba;

    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    publisher->publish(marker);

    if (!label.empty()) {
        visualization_msgs::msg::Marker text_marker;

        text_marker.header.frame_id = frame;
        text_marker.header.stamp = rclcpp::Clock().now();

        text_marker.ns = "seaweed_perception_text_labels";
        text_marker.id = id;

        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        text_marker.pose.position.x = x;
        text_marker.pose.position.y = y;
        text_marker.pose.position.z = z + 0.5;
        text_marker.pose.orientation.x = 0.0;
        text_marker.pose.orientation.y = 0.0;
        text_marker.pose.orientation.z = 0.0;
        text_marker.pose.orientation.w = 1.0;

        text_marker.scale.z = 0.1;

        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0;

        text_marker.text = label;

        text_marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        publisher->publish(text_marker);
    }
}

void reset_markers(const std::string& frame,
                   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = rclcpp::Clock().now();

    marker.ns = "seaweed_perception_labels";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    publisher->publish(marker);

    marker.ns = "seaweed_perception_text_labels";
    publisher->publish(marker);
}

}  // namespace perception_utils
#include <cv_bridge/cv_bridge.h>

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "seaweed_interfaces/msg/bounding_box.hpp"
#include "seaweed_interfaces/msg/detection.hpp"
#include "seaweed_perception/perception_utils.hpp"
#include "sensor_msgs/msg/image.h"
#include "tf2_ros/transform_broadcaster.h"

// DO PROJECTION IN DETECTION CALLBACK
// BUT MAKE SURE RGB AND DEPTH IMAGES ARE NOT TOO OLD

class BBox_Projection_Node : public rclcpp::Node {
public:
    BBox_Projection_Node()
        : Node("bbox_projection_node"),
          rgb_image_topic("/wamv/sensors/cameras/camera_sensor/optical/image_raw"),
          depth_image_topic("/wamv/sensors/cameras/camera_sensor/optical/depth"),
          camera_info_topic("/wamv/sensors/cameras/camera_sensor/optical/camera_info"),
          recieved_img(false),
          image_expiration_threshold(3.0) {
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        detection_sub = this->create_subscription<seaweed_interfaces::msg::Detection>(
            "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10,
            std::bind(&BBox_Projection_Node::detection_callback, this, std::placeholders::_1));
        rgb_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            rgb_image_topic, perception_utils::image_qos,
            std::bind(&BBox_Projection_Node::rgb_image_callback, this, std::placeholders::_1));
        depth_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            depth_image_topic, perception_utils::image_qos,
            std::bind(&BBox_Projection_Node::rgb_image_callback, this, std::placeholders::_1));
    };

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Subscription<seaweed_interfaces::msg::Detection>::SharedPtr detection_sub;
    std::string rgb_image_topic;
    std::string depth_image_topic;
    std::string camera_info_topic;

    cv::Mat latest_rgb_image;
    cv::Mat latest_depth_image;

    rclcpp::Time camera_update_timestamp;

    cv_bridge::CvImagePtr cv_ptr;
    bool recieved_img;
    float image_expiration_threshold;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub;

    void detection_callback(const seaweed_interfaces::msg::Detection::SharedPtr msg) {
        if (msg->detections.empty()) {
            RCLCPP_WARN(this->get_logger(), "got no detections");
            return;
        }
        if (!recieved_img || !is_image_valid(camera_update_timestamp, image_expiration_threshold)) {
            RCLCPP_WARN(this->get_logger(), "No valid image");
            return;
        }
    };

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

    float get_depth_at_pixel(const int x, const int y) {
        float depth_meters = latest_depth_image.at<float>(y, x);
        if (std::isfinite(depth_meters)) {
            // Gazebo/ZED use NaN/inf, need to account for both, decide if -1 or inf is the play
            return -1;
        }
        return depth_meters;
    };

    bool is_image_valid(const rclcpp::Time timestamp, float expiration_seconds) {
        rclcpp::Duration time_since_last_update = this->get_clock()->now() - timestamp;
        return time_since_last_update.seconds() <= expiration_seconds;
    };
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBox_Projection_Node>());
    rclcpp::shutdown();
    return 0;
}

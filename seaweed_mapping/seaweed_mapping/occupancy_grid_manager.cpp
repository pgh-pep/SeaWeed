#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "seaweed_perception/perception_utils.hpp"

class OccupancyGridManager : public rclcpp::Node {
public:
    OccupancyGridManager()
        : Node("occupancy_grid_manager"),
          debug(true),
          width(250),        // cells
          height(250),       // cells
          resolution(0.25),  // m/cell
          filtered_pc_topic("/pc/filtered"),
          reset_service("/mapping/costmap/reset"),
          base_link_frame("wamv/base_link"),
          costmap_topic("/mapping/costmap") {
        // local costmap: robot at center, origin offset by half grid size
        origin_x = -(width * resolution) / 2.0;
        origin_y = -(height * resolution) / 2.0;

        filtered_pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            filtered_pc_topic, 10,
            std::bind(&OccupancyGridManager::filtered_pc_callback, this, std::placeholders::_1));

        costmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic, 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/markers", 1);

        reset_srv = this->create_service<std_srvs::srv::Trigger>(
            reset_service,
            std::bind(&OccupancyGridManager::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

        map_publish_timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                                    std::bind(&OccupancyGridManager::pub_map, this));

        RCLCPP_INFO(this->get_logger(), "occupancy grid manager started");
        initialize_costmap();
    }

private:
    bool debug;
    int width;
    int height;
    double resolution;
    double origin_x;
    double origin_y;

    std::string filtered_pc_topic;
    std::string reset_service;
    std::string base_link_frame;
    std::string costmap_topic;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;
    rclcpp::TimerBase::SharedPtr map_publish_timer;

    std::vector<int8_t> costmap;
    static constexpr int8_t UNKNOWN = -1;
    static constexpr int8_t FREE = 0;
    static constexpr int8_t OCCUPIED = 100;

    void filtered_pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pc);

        if (pc->empty()) {
            RCLCPP_DEBUG(this->get_logger(), "empty pointcloud");
            return;
        }

        clear_costmap();
        project_points_to_grid(pc);
    }

    void clear_costmap() {
        std::fill(costmap.begin(), costmap.end(), FREE);  //
    }

    void project_points_to_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc) {
        for (const auto& point : pc->points) {
            int cell_x = world_to_grid_x(point.x);
            int cell_y = world_to_grid_y(point.y);

            if (in_bounds(cell_x, cell_y)) {
                cell(cell_x, cell_y) = OCCUPIED;
            }
        }
    }

    void pub_map() {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = base_link_frame;
        msg.info.resolution = resolution;
        msg.info.width = width;
        msg.info.height = height;
        msg.info.origin.position.x = origin_x;
        msg.info.origin.position.y = origin_y;
        msg.info.origin.orientation.w = 1.0;
        msg.data = costmap;
        costmap_pub->publish(msg);
    }

    void reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        initialize_costmap();
        response->success = true;
        response->message = "RESET COSTMAP";
        RCLCPP_INFO(this->get_logger(), "RESET COSTMAP");
    }

    void initialize_costmap() {
        costmap.clear();
        costmap.resize(width * height, FREE);
        RCLCPP_INFO(this->get_logger(), "init costmap: %d x %d cells", width, height);
    }

    int world_to_grid_x(double wx) {
        return static_cast<int>((wx - origin_x) / resolution);  //
    }
    int world_to_grid_y(double wy) {
        return static_cast<int>((wy - origin_y) / resolution);  //
    }

    bool in_bounds(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height;  //
    }

    int8_t& cell(int x, int y) { return costmap[y * width + x]; }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridManager>());
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "asr_summer_school/frontier_detection.h"

class FrontierDetectionNode : public rclcpp::Node
{
public:
  explicit FrontierDetectionNode(const rclcpp::NodeOptions & options)
  : Node("frontier_detection_node", options)
  {
    declare_parameter("epsilon", 0.5);
    declare_parameter("min_points", 3);
    declare_parameter("min_frontier_size", 5);
    declare_parameter("map_topic", std::string("map"));

    params_.epsilon           = get_parameter("epsilon").as_double();
    params_.min_points        = get_parameter("min_points").as_int();
    params_.min_frontier_size = get_parameter("min_frontier_size").as_int();

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "frontier_centroids", rclcpp::QoS(1).transient_local());

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(), rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        auto centroids = frontier_detection::detect_frontiers(*msg, params_);
        frontier_detection::publish_frontiers_marker(
          marker_pub_, centroids, msg->header.frame_id, get_clock());
      });
  }

private:
  frontier_detection::Params params_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(FrontierDetectionNode)

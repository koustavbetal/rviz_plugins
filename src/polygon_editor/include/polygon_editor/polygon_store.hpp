#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace polygon_editor
{

/// One process-wide instance shared by the Tool and the Panel so they
/// operate on the same vertex list. Not thread-safe beyond the mutex below;
/// all calls are expected from RViz's Qt/render thread.
class PolygonStore
{
public:
  static PolygonStore & instance();

  // Called once, lazily, the first time either the Tool or the Panel needs it.
  void ensureInitialized();

  void addVertex(const geometry_msgs::msg::Point & p);
  void clear();
  bool loadSavedPolygon(double & swath_angle_deg);
  void undoLast();

  geometry_msgs::msg::Polygon toPolygon() const;
  size_t vertexCount() const;

  /// Called by an interactive marker's feedback callback when the user
  /// drags a vertex around.
  void updateVertex(size_t index, const geometry_msgs::msg::Point & p);

  rclcpp::Node::SharedPtr node() { return node_; }

  /// First-pass "send": publishes the current polygon as a latched
  /// PolygonStamped on /coverage_polygon. Returns false if <3 vertices.
  /// Swap the body for an rclcpp_action::Client<YourAction> call once you
  /// give me the coverage action's type/name — the vertex data (toPolygon())
  /// stays the same either way.
  bool sendToCoverageServer(
  double headland_width = 0.3,
  double swath_angle_deg = 0.0,
  bool use_set_angle = false);

  /// Looks up target_frame -> source_frame via TF and records the robot's
  /// current (x, y) as the next polygon vertex. Returns false if the
  /// transform isn't available (e.g. TF tree not up yet, or a typo'd frame).
  bool recordCurrentRobotPose(
    const std::string & target_frame = "map",
    const std::string & source_frame = "base_footprint");

private:
  PolygonStore() = default;

  void publishLineStrip();
  void rebuildMarkers();
  std::string vertexMarkerName(size_t index) const;
  void onMarkerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  mutable std::mutex mutex_;
  std::vector<geometry_msgs::msg::Point> vertices_;

  bool initialized_ = false;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 
};

}  // namespace polygon_editor

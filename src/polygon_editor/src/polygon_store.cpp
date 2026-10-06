#include "polygon_editor/polygon_store.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opennav_coverage_msgs/action/compute_coverage_path.hpp>
#include <opennav_coverage_msgs/msg/coordinates.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rclcpp/qos.hpp>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace polygon_editor
{

PolygonStore & PolygonStore::instance()
{
  static PolygonStore inst;
  return inst;
}

void PolygonStore::ensureInitialized()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) {
    return;
  }

  node_ = std::make_shared<rclcpp::Node>("polygon_editor");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // spin_thread=false: we already spin node_ ourselves below, so the
  // listener's /tf and /tf_static subscriptions ride on that same spin
  // instead of a second competing executor on the same node.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "polygon_editor_markers", node_);

  line_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "coverage_polygon_outline", rclcpp::QoS(1).transient_local());

  polygon_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "coverage_polygon", rclcpp::QoS(1).transient_local());

  // Spin this node's callbacks (interactive marker feedback) on a background
  // thread so dragging works without blocking RViz's own event loop.
  std::thread([this]() {
    rclcpp::spin(node_);
  }).detach();

  initialized_ = true;
}

std::string PolygonStore::vertexMarkerName(size_t index) const
{
  return "vertex_" + std::to_string(index);
}

void PolygonStore::addVertex(const geometry_msgs::msg::Point & p)
{
  ensureInitialized();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vertices_.push_back(p);
  }
  rebuildMarkers();
  publishLineStrip();
}

void PolygonStore::clear()
{
  ensureInitialized();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vertices_.clear();
  }
  rebuildMarkers();
  publishLineStrip();
}

void PolygonStore::undoLast()
{
  ensureInitialized();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!vertices_.empty()) {
      vertices_.pop_back();
    }
  }
  rebuildMarkers();
  publishLineStrip();
}

void PolygonStore::updateVertex(size_t index, const geometry_msgs::msg::Point & p)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index >= vertices_.size()) {
      return;
    }
    vertices_[index] = p;
  }
  publishLineStrip();
}

size_t PolygonStore::vertexCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return vertices_.size();
}

geometry_msgs::msg::Polygon PolygonStore::toPolygon() const
{
  geometry_msgs::msg::Polygon poly;
  std::lock_guard<std::mutex> lock(mutex_);
  poly.points.reserve(vertices_.size());
  for (const auto & p : vertices_) {
    geometry_msgs::msg::Point32 p32;
    p32.x = static_cast<float>(p.x);
    p32.y = static_cast<float>(p.y);
    p32.z = static_cast<float>(p.z);
    poly.points.push_back(p32);
  }
  return poly;
}

void PolygonStore::onMarkerFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (feedback->event_type !=
    visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    return;
  }
  // marker names are "vertex_<N>"
  const std::string prefix = "vertex_";
  if (feedback->marker_name.rfind(prefix, 0) != 0) {
    return;
  }
  size_t index = std::stoul(feedback->marker_name.substr(prefix.size()));
  geometry_msgs::msg::Point p;
  p.x = feedback->pose.position.x;
  p.y = feedback->pose.position.y;
  p.z = feedback->pose.position.z;
  updateVertex(index, p);
}

void PolygonStore::rebuildMarkers()
{
  marker_server_->clear();

  std::vector<geometry_msgs::msg::Point> snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = vertices_;
  }

  for (size_t i = 0; i < snapshot.size(); ++i) {
    visualization_msgs::msg::InteractiveMarker im;
    im.header.frame_id = "map";
    im.name = vertexMarkerName(i);
    im.description = "";
    im.scale = 0.4f;
    im.pose.position = snapshot[i];
    im.pose.orientation.w = 1.0;

    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25;
    sphere.color.r = 1.0f;
    sphere.color.g = 0.55f;
    sphere.color.b = 0.0f;
    sphere.color.a = 1.0f;

    visualization_msgs::msg::InteractiveMarkerControl move_control;
    move_control.always_visible = true;
    move_control.markers.push_back(sphere);
    move_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    move_control.orientation.w = 1.0;
    move_control.orientation.y = 1.0;  // plane normal -> Z, so drag moves in XY
    im.controls.push_back(move_control);

    marker_server_->insert(
      im,
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & fb) {
        onMarkerFeedback(fb);
      });
  }

  marker_server_->applyChanges();
}

void PolygonStore::publishLineStrip()
{
  std::vector<geometry_msgs::msg::Point> snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = vertices_;
  }

  visualization_msgs::msg::Marker line;
  line.header.frame_id = "map";
  line.header.stamp = node_->now();
  line.ns = "coverage_polygon";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.05;
  line.color.r = 0.1f;
  line.color.g = 0.8f;
  line.color.b = 1.0f;
  line.color.a = 1.0f;
  line.pose.orientation.w = 1.0;

  line.points = snapshot;
  if (snapshot.size() > 2) {
    line.points.push_back(snapshot.front());  // close the loop visually
  }

  line_pub_->publish(line);
}
bool PolygonStore::sendToCoverageServer(
  double headland_width, double swath_angle_deg, bool use_set_angle)
{
  ensureInitialized();
  if (vertexCount() < 3) {
    RCLCPP_WARN(node_->get_logger(), "Need at least 3 vertices to send a polygon.");
    return false;
  }

  using ComputeCoveragePath = opennav_coverage_msgs::action::ComputeCoveragePath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ComputeCoveragePath>;

  static auto client = rclcpp_action::create_client<ComputeCoveragePath>(
    node_, "compute_coverage_path");

  if (!client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "compute_coverage_path action server not available.");
    return false;
  }

  ComputeCoveragePath::Goal goal;
  goal.generate_headland = true;
  goal.generate_route = true;
  goal.generate_path = true;
  goal.use_gml_file = false;
  goal.frame_id = "map";

  opennav_coverage_msgs::msg::Coordinates polygon_coords;
  const auto polygon = toPolygon();
  polygon_coords.coordinates.reserve(polygon.points.size() + 1);
  for (size_t i = 0; i < polygon.points.size(); ++i) {
    opennav_coverage_msgs::msg::Coordinate coordinate;
    coordinate.axis1 = polygon.points[i].x;
    coordinate.axis2 = polygon.points[i].y;
    polygon_coords.coordinates.push_back(coordinate);
  }
  if (polygon_coords.coordinates.front().axis1 != polygon_coords.coordinates.back().axis1 ||
    polygon_coords.coordinates.front().axis2 != polygon_coords.coordinates.back().axis2)
  {
    polygon_coords.coordinates.push_back(polygon_coords.coordinates.front());
  }
  goal.polygons.push_back(polygon_coords);

  // Match the actual generated ROS action schema in opennav_coverage_msgs.
  goal.headland_mode.mode = "CONSTANT";
  goal.headland_mode.width = static_cast<float>(headland_width);

  goal.swath_mode.objective = "LENGTH";
  goal.swath_mode.mode = use_set_angle ? "SET_ANGLE" : "BRUTE_FORCE";
  if (use_set_angle) {
    goal.swath_mode.best_angle = static_cast<float>(swath_angle_deg * M_PI / 180.0);
  } else {
    goal.swath_mode.step_angle = 0.017453f;
  }

  std::string zone_path;
  try {
    zone_path =
      ament_index_cpp::get_package_share_directory("polygon_editor") + "/config/zone.yaml";
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "Could not locate polygon_editor share directory: %s", ex.what());
    return false;
  }

  std::ofstream zone_file(zone_path, std::ios::trunc);
  if (!zone_file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not open zone file for writing: %s", zone_path.c_str());
    return false;
  }
  zone_file << std::fixed << std::setprecision(4);
  for (const auto & coordinate : goal.polygons.front().coordinates) {
    zone_file << "- {axis1: " << coordinate.axis1
              << ", axis2: " << coordinate.axis2 << "}\n";
  }
  zone_file << "\ndefault_swath_angle: " << std::setprecision(6)
            << swath_angle_deg * M_PI / 180.0 << "\n"
            << "# ------------------------------------------------\n";
  zone_file.close();
  if (!zone_file) {
    RCLCPP_ERROR(node_->get_logger(), "Failed while writing zone file: %s", zone_path.c_str());
    return false;
  }

  std::ostringstream goal_dump;
  goal_dump << "ComputeCoveragePath goal:\n"
    << "  frame_id: " << goal.frame_id << '\n'
    << "  generate_headland: " << std::boolalpha << goal.generate_headland << '\n'
    << "  generate_route: " << goal.generate_route << '\n'
    << "  generate_path: " << goal.generate_path << '\n'
    << "  use_gml_file: " << goal.use_gml_file << '\n'
    << "  gml_field: " << goal.gml_field << '\n'
    << "  headland_mode: {mode: " << goal.headland_mode.mode
    << ", width: " << goal.headland_mode.width << "}\n"
    << "  swath_mode: {objective: " << goal.swath_mode.objective
    << ", mode: " << goal.swath_mode.mode
    << ", best_angle_rad: " << goal.swath_mode.best_angle
    << ", step_angle_rad: " << goal.swath_mode.step_angle << "}\n"
    << "  row_swath_mode: {mode: " << goal.row_swath_mode.mode
    << ", offset: " << goal.row_swath_mode.offset << ", skip_ids: [";
  for (size_t i = 0; i < goal.row_swath_mode.skip_ids.size(); ++i) {
    goal_dump << (i == 0 ? "" : ", ") << goal.row_swath_mode.skip_ids[i];
  }
  goal_dump << "]}\n"
    << "  route_mode: {mode: " << goal.route_mode.mode
    << ", spiral_n: " << goal.route_mode.spiral_n << ", custom_order: [";
  for (size_t i = 0; i < goal.route_mode.custom_order.size(); ++i) {
    goal_dump << (i == 0 ? "" : ", ") << goal.route_mode.custom_order[i];
  }
  goal_dump << "]}\n"
    << "  path_mode: {mode: " << goal.path_mode.mode
    << ", continuity_mode: " << goal.path_mode.continuity_mode
    << ", turn_point_distance: " << goal.path_mode.turn_point_distance << "}\n"
    << "  polygons:\n";
  for (size_t polygon_index = 0; polygon_index < goal.polygons.size(); ++polygon_index) {
    goal_dump << "    - coordinates:\n";
    for (const auto & coordinate : goal.polygons[polygon_index].coordinates) {
      goal_dump << "        - [" << coordinate.axis1 << ", " << coordinate.axis2 << "]\n";
    }
  }
  RCLCPP_INFO(node_->get_logger(), "%s", goal_dump.str().c_str());

  rclcpp_action::Client<ComputeCoveragePath>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback =
    [this](const GoalHandle::SharedPtr & goal_handle) {
      if (goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "ComputeCoveragePath goal accepted by server.");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "ComputeCoveragePath goal rejected by server.");
      }
    };
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      if (result.result) {
        RCLCPP_INFO(
          node_->get_logger(),
          "ComputeCoveragePath result: action_code=%d error_code=%u planning_time=%d.%09u "
          "nav_path_points=%zu swaths=%zu turns=%zu",
          static_cast<int>(result.code), result.result->error_code,
          result.result->planning_time.sec, result.result->planning_time.nanosec,
          result.result->nav_path.poses.size(), result.result->coverage_path.swaths.size(),
          result.result->coverage_path.turns.size());
      }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
        result.result->error_code == ComputeCoveragePath::Result::NONE)
      {
        RCLCPP_INFO(
          node_->get_logger(), "Coverage path computed: %zu path points.",
          result.result->nav_path.poses.size());
      } else {
        RCLCPP_ERROR(
          node_->get_logger(), "Coverage path computation failed/cancelled (action_code=%d).",
          static_cast<int>(result.code));
      }
    };

  client->async_send_goal(goal, send_goal_options);
  return true;
}

bool PolygonStore::recordCurrentRobotPose(
  const std::string & target_frame, const std::string & source_frame)
{
  ensureInitialized();

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, std::chrono::milliseconds(200));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(), "Could not record robot pose (%s -> %s): %s",
      target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }

  geometry_msgs::msg::Point p;
  p.x = tf.transform.translation.x;
  p.y = tf.transform.translation.y;
  p.z = 0.0;

  addVertex(p);
  return true;
}

}  // namespace polygon_editor

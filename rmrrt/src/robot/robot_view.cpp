#include "rmrrt/robot/robot_view.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace rmrrt {
namespace robot {

namespace {

std_msgs::ColorRGBA rgba(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

const std_msgs::ColorRGBA kStart = rgba(0.2f, 0.9f, 0.2f, 1.0f);
const std_msgs::ColorRGBA kGoal = rgba(0.2f, 0.5f, 1.0f, 1.0f);
const std_msgs::ColorRGBA kStartTree = rgba(1.0f, 0.6f, 0.1f, 0.9f);
const std_msgs::ColorRGBA kGoalTree = rgba(0.3f, 0.8f, 1.0f, 0.9f);
const std_msgs::ColorRGBA kRejected = rgba(1.0f, 0.2f, 1.0f, 0.6f);
const std_msgs::ColorRGBA kPath = rgba(1.0f, 1.0f, 0.2f, 1.0f);
const std_msgs::ColorRGBA kText = rgba(1.0f, 1.0f, 1.0f, 1.0f);

geometry_msgs::Point point(const Eigen::Vector3d& p)
{
  geometry_msgs::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

}  // namespace

RobotView::RobotView(ros::NodeHandle& node, const RobotProblem& problem, const ScenarioSpec& spec)
  : problem_(problem), spec_(spec), model_(problem.scene().defaultState().getRobotModel())
{
  scene_ = node.advertise<moveit_msgs::PlanningScene>("rmrrt_view/scene", 1, true);
  start_ = node.advertise<moveit_msgs::DisplayRobotState>("rmrrt_view/start", 1, true);
  goal_ = node.advertise<moveit_msgs::DisplayRobotState>("rmrrt_view/goal", 1, true);
  path_ = node.advertise<moveit_msgs::DisplayTrajectory>("rmrrt_view/path", 1, true);
  search_ = node.advertise<visualization_msgs::MarkerArray>("rmrrt_view/search", 1, true);
  publishWorldFrame();
}

void RobotView::showQuery(ConstRefVector start, ConstRefVector goal)
{
  moveit_msgs::PlanningScene scene;
  problem_.scene().message(problem_.state(start), scene);
  scene_.publish(scene);
  publishState(start_, problem_.state(start), kStart);
  publishState(goal_, problem_.state(goal), kGoal);
  const Eigen::Vector3d a = objectPosition(start), b = objectPosition(goal);
  label_at_ = 0.5 * (a + b) + Eigen::Vector3d(0.0, 0.0, 0.5);
  markers_.markers.clear();
  addBall("start", a, 0.04, kStart);
  addBall("goal", b, 0.04, kGoal);
  publishMarkers();
  showPath({start});
}

void RobotView::showSearch(const RRTConnect& planner, const std::string& label)
{
  addEdges("start tree", planner.trace().start_edges, 0.003, kStartTree);
  addEdges("goal tree", planner.trace().goal_edges, 0.003, kGoalTree);
  addEdges("rejected", planner.trace().rejected, 0.002, kRejected);
  auto& text = addMarker("label", visualization_msgs::Marker::TEXT_VIEW_FACING, kText);
  text.pose.position = point(label_at_);
  text.scale.z = 0.06;
  text.text = label;
  publishMarkers();
}

void RobotView::showPath(const std::vector<Vector>& path)
{
  const std::vector<Vector> states = densePath(path);
  const double dt = states.size() > 1 ? duration_ / static_cast<double>(states.size() - 1) : 0.0;
  robot_trajectory::RobotTrajectory trajectory(model_);
  for (std::size_t k = 0; k < states.size(); ++k)
    trajectory.addSuffixWayPoint(problem_.state(states[k]), k ? dt : 0.0);
  moveit_msgs::DisplayTrajectory message;
  message.model_id = model_->getName();
  message.trajectory.resize(1);
  trajectory.getRobotTrajectoryMsg(message.trajectory.front());
  moveit::core::robotStateToRobotStateMsg(problem_.state(states.front()), message.trajectory_start);
  path_.publish(message);
  if (path.size() < 2)
    return;
  auto& line = addMarker("path", visualization_msgs::Marker::LINE_STRIP, kPath);
  line.scale.x = 0.008;
  for (const Vector& q : states)
    line.points.push_back(point(objectPosition(q)));
  publishMarkers();
}

Eigen::Vector3d RobotView::objectPosition(ConstRefVector q) const
{
  const auto& state = problem_.state(q);
  if (spec_.has_carried)
    return (state.getGlobalLinkTransform(spec_.carried.link) * spec_.carried.pose).translation();
  return state.getGlobalLinkTransform(spec_.tips.front()).translation();
}

visualization_msgs::Marker& RobotView::addMarker(const std::string& ns, int type, const std_msgs::ColorRGBA& color)
{
  visualization_msgs::Marker m;
  m.header.frame_id = model_->getModelFrame();
  m.ns = ns;
  m.id = static_cast<int>(markers_.markers.size());
  m.type = type;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.color = color;
  markers_.markers.push_back(m);
  return markers_.markers.back();
}

void RobotView::addEdges(const std::string& ns, const std::vector<std::pair<Vector, Vector>>& edges, double width,
                         const std_msgs::ColorRGBA& color)
{
  if (edges.empty())
    return;
  auto& m = addMarker(ns, visualization_msgs::Marker::LINE_LIST, color);
  m.scale.x = width;
  for (const auto& [from, to] : edges)
  {
    m.points.push_back(point(objectPosition(from)));
    m.points.push_back(point(objectPosition(to)));
  }
}

void RobotView::addBall(const std::string& ns, const Eigen::Vector3d& center, double diameter, const std_msgs::ColorRGBA& color)
{
  auto& m = addMarker(ns, visualization_msgs::Marker::SPHERE, color);
  m.pose.position = point(center);
  m.scale.x = m.scale.y = m.scale.z = diameter;
}

// The first marker clears what an earlier run left in RViz.
void RobotView::publishMarkers()
{
  visualization_msgs::MarkerArray message;
  message.markers.resize(1);
  message.markers.front().header.frame_id = model_->getModelFrame();
  message.markers.front().action = visualization_msgs::Marker::DELETEALL;
  message.markers.insert(message.markers.end(), markers_.markers.begin(), markers_.markers.end());
  search_.publish(message);
}

void RobotView::publishState(ros::Publisher& publisher, const moveit::core::RobotState& state, const std_msgs::ColorRGBA& color)
{
  moveit_msgs::DisplayRobotState message;
  moveit::core::robotStateToRobotStateMsg(state, message.state);
  for (const auto* link : model_->getLinkModelsWithCollisionGeometry())
  {
    moveit_msgs::ObjectColor tint;
    tint.id = link->getName();
    tint.color = color;
    message.highlight_links.push_back(tint);
  }
  publisher.publish(message);
}

// RViz uses the fixed frame "world"; the robot's root frame is put there.
void RobotView::publishWorldFrame()
{
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "world";
  transform.child_frame_id = model_->getModelFrame() == "world" ? "rmrrt_view" : model_->getModelFrame();
  transform.transform.rotation.w = 1.0;
  frames_.sendTransform(transform);
}

std::vector<Vector> RobotView::densePath(const std::vector<Vector>& path) const
{
  std::vector<Vector> states{path.front()};
  for (std::size_t k = 0; k + 1 < path.size(); ++k)
  {
    const Vector segment = path[k + 1] - path[k];
    const int pieces = std::max(1, static_cast<int>(std::ceil(segment.norm() / spacing_)));
    for (int i = 1; i <= pieces; ++i)
      states.push_back(path[k] + segment * (static_cast<double>(i) / pieces));
  }
  return states;
}

}  // namespace robot
}  // namespace rmrrt

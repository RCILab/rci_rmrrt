#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "rmrrt/planner/rrt_connect.hpp"
#include "rmrrt/robot/robot_problem.hpp"

namespace rmrrt {
namespace robot {

// A query and its search in RViz (launch/rviz/view.rviz): the scene, the start and goal states, the path
// as an animated trajectory, and the trees, rejected steps and path drawn where the carried object is.
class RobotView
{
public:
  RobotView(ros::NodeHandle& node, const RobotProblem& problem, const ScenarioSpec& spec);

  void setDuration(double seconds) { duration_ = seconds; }  // one pass of the path animation
  void setSpacing(double spacing) { spacing_ = spacing; }    // joint-space spacing of the animated states

  void showQuery(ConstRefVector start, ConstRefVector goal);
  void showSearch(const RRTConnect& planner, const std::string& label);
  // A path of one configuration shows the robot standing there.
  void showPath(const std::vector<Vector>& path);

private:
  // The center of the carried object (the first tool frame without one).
  Eigen::Vector3d objectPosition(ConstRefVector q) const;
  visualization_msgs::Marker& addMarker(const std::string& ns, int type, const std_msgs::ColorRGBA& color);
  void addEdges(const std::string& ns, const std::vector<std::pair<Vector, Vector>>& edges, double width, const std_msgs::ColorRGBA& color);
  void addBall(const std::string& ns, const Eigen::Vector3d& center, double diameter, const std_msgs::ColorRGBA& color);
  void publishMarkers();
  void publishState(ros::Publisher& publisher, const moveit::core::RobotState& state, const std_msgs::ColorRGBA& color);
  void publishWorldFrame();
  std::vector<Vector> densePath(const std::vector<Vector>& path) const;

  const RobotProblem& problem_;
  ScenarioSpec spec_;
  moveit::core::RobotModelConstPtr model_;
  ros::Publisher scene_, start_, goal_, path_, search_;
  tf2_ros::StaticTransformBroadcaster frames_;
  visualization_msgs::MarkerArray markers_;
  Eigen::Vector3d label_at_{Eigen::Vector3d::Zero()};
  double duration_{10.0};
  double spacing_{0.02};
};

}  // namespace robot
}  // namespace rmrrt

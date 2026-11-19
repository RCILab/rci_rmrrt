// One sphere instance solved by each method, side by side in RViz: the obstacles (red), the start tree
// (orange), the goal tree (light blue), the steps the validity check rejected (magenta) and the path
// (yellow). Each method is a namespace of the marker display and can be switched off there.
//
//   roslaunch rmrrt sphere_demo.launch
//   roslaunch rmrrt sphere_demo.launch layout:=gate h:=0.25 instance:=3
#include <cstdio>
#include <iomanip>
#include <sstream>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "rmrrt/util/command_line.hpp"
#include "sphere_problem.hpp"

using namespace rmrrt;
using namespace rmrrt::sphere;

namespace {

struct Color
{
  float r, g, b, a;
};

const Color kSurface{0.8f, 0.8f, 0.8f, 0.25f};
const Color kObstacle{0.85f, 0.25f, 0.25f, 0.9f};
const Color kStart{0.2f, 0.9f, 0.2f, 1.0f};
const Color kGoal{0.2f, 0.5f, 1.0f, 1.0f};
const Color kStartTree{1.0f, 0.6f, 0.1f, 0.9f};
const Color kGoalTree{0.3f, 0.8f, 1.0f, 0.9f};
const Color kRejected{1.0f, 0.2f, 1.0f, 1.0f};
const Color kPath{1.0f, 1.0f, 0.2f, 1.0f};
const Color kText{1.0f, 1.0f, 1.0f, 1.0f};

// The markers of one method around a sphere shifted by `offset` along x, in the namespace `name`.
class SphereMarkers
{
public:
  SphereMarkers(visualization_msgs::MarkerArray& markers, const std::string& name, double offset)
    : markers_(markers), name_(name), offset_(offset)
  {
  }

  void addBall(const Eigen::Vector3d& center, double radius, const Color& color)
  {
    auto& m = add(visualization_msgs::Marker::SPHERE, color);
    m.pose.position = point(center);
    m.scale.x = m.scale.y = m.scale.z = 2.0 * radius;
  }

  void addEdges(const std::vector<std::pair<Vector, Vector>>& edges, double width, const Color& color)
  {
    auto& m = add(visualization_msgs::Marker::LINE_LIST, color);
    m.scale.x = width;
    for (const auto& [from, to] : edges)
    {
      m.points.push_back(point(from));
      m.points.push_back(point(to));
    }
  }

  void addPath(const std::vector<Vector>& path, double width, const Color& color)
  {
    auto& m = add(visualization_msgs::Marker::LINE_STRIP, color);
    m.scale.x = width;
    for (const Vector& q : path)
      m.points.push_back(point(q));
  }

  void addText(const std::string& text, double height, const Color& color)
  {
    auto& m = add(visualization_msgs::Marker::TEXT_VIEW_FACING, color);
    m.pose.position = point(Eigen::Vector3d(0.0, 0.0, height));
    m.scale.z = 0.16;
    m.text = text;
  }

private:
  visualization_msgs::Marker& add(int type, const Color& color)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = "sphere_demo";
    m.ns = name_;
    m.id = static_cast<int>(markers_.markers.size());
    m.type = type;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = color.r;
    m.color.g = color.g;
    m.color.b = color.b;
    m.color.a = color.a;
    markers_.markers.push_back(m);
    return markers_.markers.back();
  }

  geometry_msgs::Point point(ConstRefVector q) const
  {
    geometry_msgs::Point p;
    p.x = q[0] + offset_;
    p.y = q[1];
    p.z = q[2];
    return p;
  }

  visualization_msgs::MarkerArray& markers_;
  std::string name_;
  double offset_;
};

}  // namespace

int run(int argc, char** argv)
{
  po::options_description options("usage: rosrun rmrrt sphere_demo [options]");
  options.add_options()
    ("layout", po::value<std::string>()->default_value("double_gate"), "open | clutter | gate | double_gate")
    ("h", po::value<double>()->default_value(0.15), "half-width of the openings (gate, double_gate)")
    ("instance", po::value<int>()->default_value(5), "instance number, as in sphere_benchmark");
  const po::variables_map args = parseCommandLine(argc, argv, options);
  const int s = args["instance"].as<int>();
  const SphereInstance instance = makeInstance(args["layout"].as<std::string>(), args["h"].as<double>(), s);
  const SphereProblem problem(instance.centers, instance.radii);
  const RiemannianMetric metric;

  visualization_msgs::MarkerArray markers;
  std::printf("  %-9s  %6s  %8s  %8s  %6s\n", "method", "solved", "checks", "rejected", "path");
  const std::vector<Method>& list = methods();
  for (std::size_t k = 0; k < list.size(); ++k)
  {
    RRTConnect planner(problem, metric);
    setUp(planner, list[k], s);
    planner.setRecording(true);
    const bool solved = planner.solve(instance.start, instance.goal, 300.0);
    const PlannerStatistics& stats = planner.statistics();

    SphereMarkers drawn(markers, list[k].name, 2.6 * (static_cast<double>(k) - 0.5 * static_cast<double>(list.size() - 1)));
    drawn.addBall(Eigen::Vector3d::Zero(), 1.0, kSurface);
    for (std::size_t i = 0; i < instance.centers.size(); ++i)
      drawn.addBall(instance.centers[i], instance.radii[i], kObstacle);
    drawn.addBall(instance.start, 0.035, kStart);
    drawn.addBall(instance.goal, 0.035, kGoal);
    drawn.addEdges(planner.trace().start_edges, 0.004, kStartTree);
    drawn.addEdges(planner.trace().goal_edges, 0.004, kGoalTree);
    drawn.addEdges(planner.trace().rejected, 0.006, kRejected);
    drawn.addPath(planner.path(), 0.015, kPath);
    std::ostringstream label;
    label << std::fixed << std::setprecision(2) << list[k].name << "\nchecks " << stats.checks << "   rejected " << stats.rejected << "\n";
    if (solved)
      label << "path " << planner.pathLength();
    else
      label << "not solved";
    drawn.addText(label.str(), 1.45, kText);
    std::printf("  %-9s  %6s  %8zu  %8zu  %6.2f\n", list[k].name.c_str(), solved ? "yes" : "no", stats.checks, stats.rejected, planner.pathLength());
  }

  ros::NodeHandle node;
  ros::Publisher publisher = node.advertise<visualization_msgs::MarkerArray>("sphere_demo", 1, true);
  publisher.publish(markers);
  std::printf("Shown in RViz; Ctrl-C to quit.\n");
  std::fflush(stdout);
  ros::spin();
  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sphere_demo");
  try
  {
    return run(argc, argv);
  }
  catch (const std::exception& error)
  {
    std::fprintf(stderr, "%s\n", error.what());
    return 1;
  }
}

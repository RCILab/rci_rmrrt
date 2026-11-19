// Solves one query and shows it in RViz: the scene, the start (green) and goal (blue), the path
// animated, and the start tree (orange), goal tree (light blue), rejected steps (magenta) and path
// (yellow) drawn where the carried object is.
//
//   roslaunch rmrrt view.launch scenario:=panda_dual
//   rosrun rmrrt view -E panda_dual --scene 526
//   rosrun rmrrt view -E panda_dual --scene 526 -M no_metric
#include <cstdio>
#include <sstream>

#include <ros/ros.h>

#include "rmrrt/robot/experiment.hpp"
#include "rmrrt/robot/planner_config.hpp"
#include "rmrrt/robot/robot_view.hpp"
#include "rmrrt/util/command_line.hpp"

using namespace rmrrt;
using namespace rmrrt::robot;

int run(int argc, char** argv)
{
  po::options_description options("usage: rosrun rmrrt view -E <scenario> [--scene <number>] [-M <method>] [options]");
  options.add_options()
    ("scenario,E", po::value<std::string>()->required(), "panda_orientation | panda_dual | panda_dual_orientation | panda_triple | h1_2_dual")
    ("scene", po::value<int>(), "query number (default: the first of the scenario)")
    ("method,M", po::value<std::string>()->default_value("rmrrt"), "rmrrt | steer | select | no_metric")
    ("duration", po::value<double>()->default_value(10.0), "seconds of one pass of the path animation");
  const po::variables_map args = parseCommandLine(argc, argv, options);

  const Experiment experiment(args["scenario"].as<std::string>());
  const PlannerConfig config = PlannerConfig::load(Experiment::packagePath("config/planner.yaml"));
  const Method& method = findMethod(args["method"].as<std::string>());
  const int scene = args.count("scene") ? args["scene"].as<int>() : experiment.scenes().front();
  const auto model = experiment.loadRobot();
  const Vector start = experiment.start(scene), goal = experiment.goal(scene);
  const RobotProblem problem(model, experiment.spec(), experiment.sceneFile(scene), start, goal);

  ros::NodeHandle node;
  RobotView view(node, problem, experiment.spec());
  view.setDuration(args["duration"].as<double>());
  view.showQuery(start, goal);

  RiemannianMetric metric;
  config.apply(metric);
  RRTConnect planner(problem, metric);
  config.apply(planner, method);
  planner.setSeed(Experiment::seed(config.seed, scene));
  planner.setRecording(true);
  const bool solved = planner.solve(start, goal, config.timeout);
  const PlannerStatistics& stats = planner.statistics();

  std::ostringstream label;
  label.precision(2);
  label << std::fixed << method.name << "   scene " << scene << "\n"
        << (solved ? "solved" : "not solved") << " in " << planner.seconds() << " s   checks " << stats.checks << "   rejected " << stats.rejected;
  if (solved)
    label << "   path " << planner.pathLength();
  view.showSearch(planner, label.str());
  view.showPath(solved ? planner.path() : std::vector<Vector>{start});
  std::printf("%s scene_%04d %s: %s in %.3f s, checks %zu, rejected %zu, path %.2f. Shown in RViz; Ctrl-C to quit.\n",
              experiment.spec().name.c_str(), scene, method.name.c_str(), solved ? "solved" : "not solved", planner.seconds(), stats.checks,
              stats.rejected, planner.pathLength());
  std::fflush(stdout);
  ros::spin();
  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmrrt_view", ros::init_options::AnonymousName);
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

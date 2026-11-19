// Solves the queries of a scenario and writes results/<scenario>/<method>.csv.
//
//   roslaunch rmrrt robot.launch scenario:=panda_dual
//   rosrun rmrrt benchmark -E panda_dual            # RMRRT
//   rosrun rmrrt benchmark -E panda_dual -M all     # no_metric, steer, select and rmrrt
#include <cstdio>
#include <fstream>
#include <vector>

#include <ros/ros.h>

#include "rmrrt/robot/experiment.hpp"
#include "rmrrt/robot/planner_config.hpp"
#include "rmrrt/robot/robot_problem.hpp"
#include "rmrrt/util/command_line.hpp"

using namespace rmrrt;
using namespace rmrrt::robot;

namespace {

struct Summary
{
  std::string method;
  int queries{0}, solved{0};
  double seconds{0.0}, checks{0.0}, path_length{0.0};
};

Summary runMethod(const Experiment& experiment, const moveit::core::RobotModelConstPtr& model, const PlannerConfig& config,
                  const Method& method, const std::vector<int>& scenes)
{
  RiemannianMetric metric;
  config.apply(metric);
  std::ofstream csv(experiment.resultPath(method.name + ".csv"));
  csv << "scene,solved,seconds,checks,path_length,iterations,nodes,rejected,declined\n";
  Summary summary{method.name};
  for (int scene : scenes)
  {
    const Vector start = experiment.start(scene), goal = experiment.goal(scene);
    const RobotProblem problem(model, experiment.spec(), experiment.sceneFile(scene), start, goal);
    RRTConnect planner(problem, metric);
    config.apply(planner, method);
    planner.setSeed(Experiment::seed(config.seed, scene));
    const bool solved = planner.solve(start, goal, config.timeout);
    const PlannerStatistics& stats = planner.statistics();
    std::printf("  scene_%04d  %-9s  %-6s  %7.3f s  checks %7zu  path %6.2f\n", scene, method.name.c_str(), solved ? "solved" : "failed",
                planner.seconds(), stats.checks, planner.pathLength());
    std::fflush(stdout);
    csv << scene << "," << solved << "," << planner.seconds() << "," << stats.checks << "," << planner.pathLength() << ","
        << stats.iterations << "," << stats.nodes << "," << stats.rejected << "," << stats.declined << "\n";
    ++summary.queries;
    summary.solved += solved ? 1 : 0;
    summary.seconds += solved ? planner.seconds() : config.timeout;
    summary.checks += static_cast<double>(stats.checks);
    summary.path_length += solved ? planner.pathLength() : 0.0;
  }
  return summary;
}

void printSummaries(const std::string& scenario, const std::vector<Summary>& summaries)
{
  std::printf("\n%s: means over the queries (time of a failed query = timeout; path over the solved ones)\n", scenario.c_str());
  std::printf("  %-9s  %8s  %9s  %9s  %7s\n", "method", "success", "time [s]", "checks", "path");
  for (const Summary& s : summaries)
    std::printf("  %-9s  %7.0f%%  %9.3f  %9.0f  %7.2f\n", s.method.c_str(), 100.0 * s.solved / s.queries, s.seconds / s.queries,
                s.checks / s.queries, s.solved ? s.path_length / s.solved : 0.0);
}

}  // namespace

int run(int argc, char** argv)
{
  po::options_description options("usage: rosrun rmrrt benchmark -E <scenario> [-M <method>] [options]");
  options.add_options()
    ("scenario,E", po::value<std::string>()->required(), "panda_orientation | panda_dual | panda_dual_orientation | panda_triple | h1_2_dual")
    ("method,M", po::value<std::string>()->default_value("rmrrt"), "rmrrt | steer | select | no_metric | all")
    ("scene", po::value<int>(), "only this query (default: every query of the scenario)")
    ("timeout", po::value<double>(), "seconds per query (default: config/planner.yaml)")
    ("seed", po::value<unsigned>(), "random seed (default: config/planner.yaml)");
  const po::variables_map args = parseCommandLine(argc, argv, options);

  const Experiment experiment(args["scenario"].as<std::string>());
  PlannerConfig config = PlannerConfig::load(Experiment::packagePath("config/planner.yaml"));
  if (args.count("timeout"))
    config.timeout = args["timeout"].as<double>();
  if (args.count("seed"))
    config.seed = args["seed"].as<unsigned>();
  const std::vector<int> scenes = args.count("scene") ? std::vector<int>{args["scene"].as<int>()} : experiment.scenes();
  const std::string name = args["method"].as<std::string>();
  const std::vector<Method> chosen = name == "all" ? methods() : std::vector<Method>{findMethod(name)};
  const auto model = experiment.loadRobot();

  std::vector<Summary> summaries;
  for (const Method& method : chosen)
  {
    std::printf("%s, %s: %zu queries\n", experiment.spec().name.c_str(), method.name.c_str(), scenes.size());
    summaries.push_back(runMethod(experiment, model, config, method, scenes));
  }
  printSummaries(experiment.spec().name, summaries);
  std::printf("\nresults: %s\n", experiment.resultPath("").c_str());
  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmrrt_benchmark", ros::init_options::AnonymousName);
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

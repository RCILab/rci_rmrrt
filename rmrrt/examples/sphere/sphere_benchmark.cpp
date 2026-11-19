// The four methods on sphere instances: means over the instances and, against no_metric, how many
// instances each method solves with fewer (W) or more (L) validity checks.
//
//   rosrun rmrrt sphere_benchmark
//   rosrun rmrrt sphere_benchmark --layout gate --h 0.25 --instances 50
#include <cstdio>
#include <vector>

#include "rmrrt/util/command_line.hpp"
#include "sphere_problem.hpp"

using namespace rmrrt;
using namespace rmrrt::sphere;

int main(int argc, char** argv)
{
  po::options_description options("usage: rosrun rmrrt sphere_benchmark [options]");
  options.add_options()
    ("layout", po::value<std::string>()->default_value("double_gate"), "open | clutter | gate | double_gate")
    ("h", po::value<double>()->default_value(0.15), "half-width of the openings (gate, double_gate)")
    ("instances", po::value<int>()->default_value(100), "number of instances");
  const po::variables_map args = parseCommandLine(argc, argv, options);
  const std::string layout = args["layout"].as<std::string>();
  const double h = args["h"].as<double>();
  const int instances = args["instances"].as<int>();

  const RiemannianMetric metric;
  std::printf("%s, h = %.2f: %d instances\n\n", layout.c_str(), h, instances);
  std::printf("  %-9s  %7s  %9s  %8s  %6s  %8s  %9s\n", "method", "success", "time [ms]", "checks", "path", "rejected", "W/L");
  std::vector<std::size_t> baseline;
  for (const Method& method : methods())
  {
    int solved = 0, wins = 0, losses = 0;
    double milliseconds = 0.0, checks = 0.0, path = 0.0, rejected = 0.0;
    for (int s = 0; s < instances; ++s)
    {
      const SphereInstance instance = makeInstance(layout, h, s);
      const SphereProblem problem(instance.centers, instance.radii);
      RRTConnect planner(problem, metric);
      setUp(planner, method, s);
      const bool found = planner.solve(instance.start, instance.goal, 300.0);
      const PlannerStatistics& stats = planner.statistics();
      solved += found ? 1 : 0;
      milliseconds += 1e3 * planner.seconds();
      checks += static_cast<double>(stats.checks);
      rejected += static_cast<double>(stats.rejected);
      path += found ? planner.pathLength() : 0.0;
      if (method.name == "no_metric")
        baseline.push_back(stats.checks);
      wins += stats.checks < baseline[static_cast<std::size_t>(s)] ? 1 : 0;
      losses += stats.checks > baseline[static_cast<std::size_t>(s)] ? 1 : 0;
    }
    const std::string versus = method.name == "no_metric" ? "-" : std::to_string(wins) + "/" + std::to_string(losses);
    std::printf("  %-9s  %6.0f%%  %9.2f  %8.0f  %6.2f  %8.0f  %9s\n", method.name.c_str(), 100.0 * solved / instances, milliseconds / instances,
                checks / instances, solved ? path / solved : 0.0, rejected / instances, versus.c_str());
  }
  return 0;
}

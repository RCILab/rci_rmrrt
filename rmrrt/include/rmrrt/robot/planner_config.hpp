#pragma once

#include <string>

#include "rmrrt/planner/method.hpp"

namespace rmrrt {
namespace robot {

// The parameters of config/planner.yaml, the same for every scenario.
struct PlannerConfig
{
  double step_size{0.0};
  double connection_tolerance{0.0};
  double projection_tolerance{0.0};
  double edge_resolution{0.0};
  double timeout{0.0};
  unsigned seed{0};
  double lambda{0.0};
  double beta{0.0};
  double obstacle_width{0.0};
  double self_collision_width{0.0};
  double joint_limit_width{0.0};
  double epsilon{0.0};
  double psi_max{0.0};

  static PlannerConfig load(const std::string& file);
  void apply(RiemannianMetric& metric) const;
  void apply(RRTConnect& planner, const Method& method) const;
};

}  // namespace robot
}  // namespace rmrrt

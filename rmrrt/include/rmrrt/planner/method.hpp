#pragma once

#include <string>
#include <vector>

#include "rmrrt/planner/rrt_connect.hpp"

namespace rmrrt {

// A planner variant: its steering and its node selection.
//
//   method       steering     node selection
//   no_metric    euclidean    nearest
//   steer        centered     nearest
//   select       euclidean    riemannian
//   rmrrt        centered     riemannian
struct Method
{
  std::string name;
  std::string steering;
  std::string selection;
};

const std::vector<Method>& methods();
const Method& findMethod(const std::string& name);

// Gives the planner the steering and node selection of the method. Centered steering falls back to the
// metric steering when a step is rejected; beta = 0 is the metric steering alone.
void configure(RRTConnect& planner, const Method& method, double beta);

}  // namespace rmrrt

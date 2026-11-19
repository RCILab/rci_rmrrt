#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "rmrrt/core/types.hpp"

namespace rmrrt {

struct PlannerStatistics
{
  std::size_t iterations{0};
  std::size_t nodes{0};
  std::size_t checks{0};     // calls of ProblemBase::isValid
  std::size_t rejected{0};   // steps rejected by the validity check
  std::size_t declined{0};   // extensions the node selection declined, at no validity check
  std::size_t fallbacks{0};  // rejected centered steps tried again with the metric steering
};

// What one search explored, kept when recording: the tree edges (node, parent) and the rejected
// steps (node, attempted configuration).
struct SearchTrace
{
  std::vector<std::pair<Vector, Vector>> start_edges;
  std::vector<std::pair<Vector, Vector>> goal_edges;
  std::vector<std::pair<Vector, Vector>> rejected;
};

}  // namespace rmrrt

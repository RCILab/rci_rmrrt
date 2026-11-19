#pragma once

#include "rmrrt/selection/selection_base.hpp"

namespace rmrrt {

// The Euclidean nearest node, always extended.
class SelectionNearest : public SelectionBase
{
public:
  int select(MotionTree& tree, ConstRefVector target) override { return nearest(tree, target); }
  bool accepts(MotionTree& /*tree*/, int /*i*/, ConstRefVector /*target*/) override { return true; }
  std::string name() const override { return "nearest"; }
};

}  // namespace rmrrt

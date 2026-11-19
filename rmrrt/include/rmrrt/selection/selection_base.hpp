#pragma once

#include <string>

#include "rmrrt/planner/motion_tree.hpp"

namespace rmrrt {

// Which node of a tree extends toward a target.
class SelectionBase
{
public:
  virtual ~SelectionBase() = default;

  // The node to extend, or -1 to extend none.
  virtual int select(MotionTree& tree, ConstRefVector target) = 0;
  // Whether node i extends toward the target. The planner asks before each connect step after the first.
  virtual bool accepts(MotionTree& tree, int i, ConstRefVector target) = 0;
  virtual std::string name() const = 0;
  virtual void setStepSize(double /*eta*/) {}

protected:
  // The Euclidean nearest node.
  static int nearest(const MotionTree& tree, ConstRefVector target);
};

}  // namespace rmrrt

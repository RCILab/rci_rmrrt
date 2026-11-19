#pragma once

#include <vector>

#include "rmrrt/core/types.hpp"

namespace rmrrt {

enum class Category
{
  Obstacle,
  JointLimit,
  SelfCollision
};

// One inequality h(q) > 0 close to its bound: its value and gradient at q.
struct Margin
{
  double h{0.0};
  Vector gradient;
  Category category{Category::Obstacle};
};

using Margins = std::vector<Margin>;

}  // namespace rmrrt

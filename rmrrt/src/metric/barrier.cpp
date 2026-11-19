#include "rmrrt/metric/barrier.hpp"

#include <algorithm>
#include <cmath>

namespace rmrrt {

Barrier::Barrier()
{
  setWidth(Category::Obstacle, 0.15);
  setWidth(Category::JointLimit, 0.20);
  setWidth(Category::SelfCollision, 0.05);
}

void Barrier::setWidth(Category category, double delta)
{
  delta_[index(category)] = delta;
  alpha_[index(category)] = delta > 0.0 ? delta * delta : 0.0;
}

double Barrier::weight(const Margin& margin) const
{
  const std::size_t c = index(margin.category);
  if (alpha_[c] <= 0.0 || margin.h >= delta_[c])
    return 0.0;
  const double h = std::max(margin.h, 0.0);
  return alpha_[c] * std::min(1.0 / (h * h + epsilon_ * epsilon_), psi_max_);
}

double Barrier::push(const Margin& margin) const
{
  const double w = weight(margin);
  if (w <= 0.0)
    return 0.0;
  const std::size_t c = index(margin.category);
  return std::max(std::sqrt(alpha_[c] * w) - alpha_[c] / delta_[c], 0.0);
}

}  // namespace rmrrt

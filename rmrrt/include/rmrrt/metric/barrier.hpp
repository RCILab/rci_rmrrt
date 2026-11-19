#pragma once

#include <array>

#include "rmrrt/core/margin.hpp"

namespace rmrrt {

// Log barrier of the inequalities, one width delta per category (alpha = delta^2):
//   weight  w(h) = alpha min(1 / (h^2 + eps^2), psi_max)       for h < delta, else 0
//   push    c(h) = sqrt(alpha w(h)) - alpha / delta             (-d phi / dh, zero at h = delta)
class Barrier
{
public:
  Barrier();

  void setWidth(Category category, double delta);  // 0 removes the category
  void setEpsilon(double epsilon) { epsilon_ = epsilon; }
  void setPsiMax(double psi_max) { psi_max_ = psi_max; }

  double width(Category category) const { return delta_[index(category)]; }
  bool enabled(Category category) const { return alpha_[index(category)] > 0.0; }

  double weight(const Margin& margin) const;
  double push(const Margin& margin) const;

private:
  static std::size_t index(Category category) { return static_cast<std::size_t>(category); }

  std::array<double, 3> delta_{};
  std::array<double, 3> alpha_{};
  double epsilon_{1e-3};
  double psi_max_{1e4};
};

}  // namespace rmrrt

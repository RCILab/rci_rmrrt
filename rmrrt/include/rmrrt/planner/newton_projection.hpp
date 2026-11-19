#pragma once

#include "rmrrt/metric/riemannian_metric.hpp"

namespace rmrrt {

// Minimum-G-norm Newton correction onto g(q) = 0, with G at the current iterate:
//   q <- q - G^-1 J^T (J G^-1 J^T)^-1 g(q)
class NewtonProjection
{
public:
  NewtonProjection(const ProblemBase& problem, const RiemannianMetric& metric) : problem_(problem), metric_(metric) {}

  // False when ||g(q)|| < tau is not reached.
  bool project(RefVector q, bool use_barrier) const;

  void setTolerance(double tau) { tau_ = tau; }
  void setMaxIterations(int iterations) { max_iterations_ = iterations; }
  void setMaxStep(double max_step) { max_step_ = max_step; }
  double tolerance() const { return tau_; }

private:
  const ProblemBase& problem_;
  const RiemannianMetric& metric_;
  double tau_{2e-3};
  int max_iterations_{30};
  double max_step_{0.25};
};

}  // namespace rmrrt

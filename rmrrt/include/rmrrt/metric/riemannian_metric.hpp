#pragma once

#include "rmrrt/core/problem_base.hpp"
#include "rmrrt/metric/barrier.hpp"

namespace rmrrt {

// The local geometry at a configuration q:
//   G = lambda I + sum_i w(h_i) grad h_i grad h_i^T     barrier metric in the joint space
//   P = I - G^-1 J^T (J G^-1 J^T)^-1 J                   G-orthogonal projector onto the tangent space
//   Q = G^-1 - G^-1 J^T (J G^-1 J^T)^-1 J G^-1           inverse of M = P^T G P on the tangent space
//   force = sum_i c(h_i) grad h_i                        push away from the nearby bounds
struct Chart
{
  Matrix G;
  Matrix P;
  Matrix Q;
  Vector force;
};

class RiemannianMetric
{
public:
  void setLambda(double lambda) { lambda_ = lambda; }
  Barrier& barrier() { return barrier_; }
  const Barrier& barrier() const { return barrier_; }

  // Without the barrier, G = lambda I and the force is zero. False when J has lost rank.
  bool computeChart(const ProblemBase& problem, ConstRefVector q, bool use_barrier, Chart& chart) const;
  void computeG(const ProblemBase& problem, ConstRefVector q, bool use_barrier, Matrix& G) const;

private:
  void addWeights(const Margins& margins, Matrix& G) const;
  void addPush(const Margins& margins, Vector& force) const;

  double lambda_{1.0};
  Barrier barrier_;
};

}  // namespace rmrrt

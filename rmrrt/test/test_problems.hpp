#pragma once

// Small problems with known answers for the tests.

#include "rmrrt/core/problem_base.hpp"

namespace rmrrt {
namespace test {

// Two orthonormal vectors in R^3: n = 6, m = 3, nonlinear. The given margins are returned at every q.
class OrthonormalPair : public ProblemBase
{
public:
  explicit OrthonormalPair(Margins margins = {})
    : margins_(std::move(margins)), lower_(Vector::Constant(6, -1.5)), upper_(Vector::Constant(6, 1.5))
  {
  }

  int dim() const override { return 6; }
  int coDim() const override { return 3; }
  const Vector& lowerBound() const override { return lower_; }
  const Vector& upperBound() const override { return upper_; }
  void function(ConstRefVector q, RefVector g) const override
  {
    const Eigen::Vector3d a = q.head<3>(), b = q.tail<3>();
    g << 0.5 * (a.squaredNorm() - 1.0), 0.5 * (b.squaredNorm() - 1.0), a.dot(b);
  }
  void jacobian(ConstRefVector q, RefMatrix J) const override
  {
    J.setZero();
    J.block<1, 3>(0, 0) = q.head<3>().transpose();
    J.block<1, 3>(1, 3) = q.tail<3>().transpose();
    J.block<1, 3>(2, 0) = q.tail<3>().transpose();
    J.block<1, 3>(2, 3) = q.head<3>().transpose();
  }
  void computeMargins(ConstRefVector, const Barrier&, Margins& margins) const override
  {
    margins.insert(margins.end(), margins_.begin(), margins_.end());
  }
  bool isValid(ConstRefVector) const override { return true; }

private:
  Margins margins_;
  Vector lower_, upper_;
};

inline Margin obstacleMargin(double h, const Vector& gradient)
{
  Margin margin;
  margin.h = h;
  margin.gradient = gradient;
  margin.category = Category::Obstacle;
  return margin;
}

// A point on the unit circle of the first two coordinates in R^3 (orthonormal pair start).
inline Vector orthonormalStart()
{
  Vector q(6);
  q << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  return q;
}

}  // namespace test
}  // namespace rmrrt

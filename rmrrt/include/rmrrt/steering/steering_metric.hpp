#pragma once

#include "rmrrt/steering/steering_base.hpp"

namespace rmrrt {

// v = Q P (target - q): the direction of least M-energy per unit progress toward the target.
class SteeringMetric : public SteeringBase
{
public:
  const Vector& compute(const Chart& chart, ConstRefVector q, ConstRefVector target) override;
  bool usesBarrier() const override { return true; }
  std::string name() const override { return "metric"; }
};

// The same rule on the Euclidean chart (G = lambda I): the tangent projection of target - q.
class SteeringEuclidean : public SteeringMetric
{
public:
  bool usesBarrier() const override { return false; }
  std::string name() const override { return "euclidean"; }
};

}  // namespace rmrrt

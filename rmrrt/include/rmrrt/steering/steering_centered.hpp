#pragma once

#include "rmrrt/steering/steering_base.hpp"

namespace rmrrt {

// v = Q (eta P w / |P w| + beta force),  w = target - q: the metric direction toward the target, centered
// between the nearby bounds by the first derivative of the barrier. beta = 0 is SteeringMetric.
class SteeringCentered : public SteeringBase
{
public:
  explicit SteeringCentered(double beta = 0.5) : beta_(beta) {}

  const Vector& compute(const Chart& chart, ConstRefVector q, ConstRefVector target) override;
  bool usesBarrier() const override { return true; }
  std::string name() const override { return "centered"; }

  void setBeta(double beta) { beta_ = beta; }
  double beta() const { return beta_; }

private:
  double beta_;
  Vector tangent_;
};

}  // namespace rmrrt

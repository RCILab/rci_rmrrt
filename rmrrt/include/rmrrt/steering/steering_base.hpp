#pragma once

#include <string>

#include "rmrrt/metric/riemannian_metric.hpp"

namespace rmrrt {

// The direction of one step from q toward a target, given the chart at q. Every step has length eta.
class SteeringBase
{
public:
  virtual ~SteeringBase() = default;

  // The direction v, not normalized; zero when the target has no tangent component.
  virtual const Vector& compute(const Chart& chart, ConstRefVector q, ConstRefVector target) = 0;
  // Whether the chart and the projection of this steering carry the barrier.
  virtual bool usesBarrier() const = 0;
  virtual std::string name() const = 0;

  // next = q + eta v / |v|. False without a direction.
  bool step(const Chart& chart, ConstRefVector q, ConstRefVector target, Vector& next);

  void setStepSize(double eta) { eta_ = eta; }
  double stepSize() const { return eta_; }

protected:
  double eta_{0.2};
  Vector direction_;
};

}  // namespace rmrrt

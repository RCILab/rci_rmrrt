#include "rmrrt/steering/steering_centered.hpp"
#include "rmrrt/steering/steering_metric.hpp"

namespace rmrrt {

bool SteeringBase::step(const Chart& chart, ConstRefVector q, ConstRefVector target, Vector& next)
{
  const Vector& v = compute(chart, q, target);
  const double norm = v.norm();
  if (norm < 1e-12)
    return false;
  next = q + (eta_ / norm) * v;
  return true;
}

const Vector& SteeringMetric::compute(const Chart& chart, ConstRefVector q, ConstRefVector target)
{
  direction_ = chart.Q * (chart.P * (target - q));
  return direction_;
}

const Vector& SteeringCentered::compute(const Chart& chart, ConstRefVector q, ConstRefVector target)
{
  tangent_ = chart.P * (target - q);
  const double norm = tangent_.norm();
  if (norm < 1e-12)
  {
    direction_ = Vector::Zero(q.size());
    return direction_;
  }
  const Vector toward = chart.Q * ((eta_ / norm) * tangent_);
  const Vector push = beta_ * (chart.Q * chart.force);
  direction_ = toward + push;
  return direction_;
}

}  // namespace rmrrt

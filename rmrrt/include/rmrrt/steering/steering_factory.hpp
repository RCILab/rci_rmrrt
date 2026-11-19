#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "rmrrt/steering/steering_centered.hpp"
#include "rmrrt/steering/steering_metric.hpp"

namespace rmrrt {

// type: euclidean | metric | centered (beta is used by centered only)
inline std::shared_ptr<SteeringBase> createSteering(const std::string& type, double beta = 0.5)
{
  if (type == "euclidean")
    return std::make_shared<SteeringEuclidean>();
  if (type == "metric")
    return std::make_shared<SteeringMetric>();
  if (type == "centered")
    return std::make_shared<SteeringCentered>(beta);
  throw std::invalid_argument("Unknown steering " + type + " (euclidean | metric | centered)");
}

}  // namespace rmrrt

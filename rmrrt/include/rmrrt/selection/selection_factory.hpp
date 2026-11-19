#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "rmrrt/selection/selection_nearest.hpp"
#include "rmrrt/selection/selection_riemannian.hpp"
#include "rmrrt/steering/steering_factory.hpp"

namespace rmrrt {

// type: nearest | riemannian. The Riemannian selection predicts the centered step (the metric step for
// beta = 0).
inline std::shared_ptr<SelectionBase> createSelection(const std::string& type, double beta = 0.5)
{
  if (type == "nearest")
    return std::make_shared<SelectionNearest>();
  if (type == "riemannian")
    return std::make_shared<SelectionRiemannian>(createSteering(beta > 0.0 ? "centered" : "metric", beta));
  throw std::invalid_argument("Unknown node selection " + type + " (nearest | riemannian)");
}

}  // namespace rmrrt

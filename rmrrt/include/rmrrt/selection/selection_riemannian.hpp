#pragma once

#include <memory>

#include "rmrrt/selection/selection_base.hpp"
#include "rmrrt/steering/steering_base.hpp"

namespace rmrrt {

// Riemannian node selection. The Euclidean nearest node keeps the Voronoi bias of RRT; it is extended
// only when the metric step predicted from it gets closer to the target. Otherwise no node is
// extended, so a step the metric already sees going nowhere costs no validity check.
class SelectionRiemannian : public SelectionBase
{
public:
  explicit SelectionRiemannian(std::shared_ptr<SteeringBase> predictor) : predictor_(std::move(predictor)) {}

  int select(MotionTree& tree, ConstRefVector target) override;
  bool accepts(MotionTree& tree, int i, ConstRefVector target) override;
  std::string name() const override { return "riemannian"; }
  void setStepSize(double eta) override { predictor_->setStepSize(eta); }

private:
  std::shared_ptr<SteeringBase> predictor_;
  Vector next_;
};

}  // namespace rmrrt

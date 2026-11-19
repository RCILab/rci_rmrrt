#include "rmrrt/selection/selection_riemannian.hpp"

namespace rmrrt {

int SelectionBase::nearest(const MotionTree& tree, ConstRefVector target)
{
  int best = 0;
  double best_distance = (tree.q(0) - target).squaredNorm();
  for (int i = 1; i < static_cast<int>(tree.size()); ++i)
  {
    const double distance = (tree.q(i) - target).squaredNorm();
    if (distance < best_distance)
    {
      best = i;
      best_distance = distance;
    }
  }
  return best;
}

int SelectionRiemannian::select(MotionTree& tree, ConstRefVector target)
{
  const int i = nearest(tree, target);
  return accepts(tree, i, target) ? i : -1;
}

bool SelectionRiemannian::accepts(MotionTree& tree, int i, ConstRefVector target)
{
  const Chart* chart = tree.chart(i, predictor_->usesBarrier());
  if (!chart)
    return true;  // the planner rejects the step itself
  if (!predictor_->step(*chart, tree.q(i), target, next_))
    return false;
  return (next_ - target).squaredNorm() < (tree.q(i) - target).squaredNorm();
}

}  // namespace rmrrt

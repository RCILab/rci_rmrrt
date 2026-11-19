#include "rmrrt/planner/rrt_connect.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "rmrrt/selection/selection_riemannian.hpp"
#include "rmrrt/steering/steering_centered.hpp"
#include "rmrrt/steering/steering_metric.hpp"

namespace rmrrt {

RRTConnect::RRTConnect(const ProblemBase& problem, const RiemannianMetric& metric)
  : problem_(problem), metric_(metric), projection_(problem, metric)
{
  setSteering(std::make_shared<SteeringCentered>());
  setFallback(std::make_shared<SteeringMetric>());
  setSelection(std::make_shared<SelectionRiemannian>(std::make_shared<SteeringCentered>()));
}

void RRTConnect::setSteering(std::shared_ptr<SteeringBase> steering)
{
  steering_ = std::move(steering);
  steering_->setStepSize(eta_);
}

void RRTConnect::setFallback(std::shared_ptr<SteeringBase> steering)
{
  fallback_ = std::move(steering);
  if (fallback_)
    fallback_->setStepSize(eta_);
}

void RRTConnect::setSelection(std::shared_ptr<SelectionBase> selection)
{
  selection_ = std::move(selection);
  selection_->setStepSize(eta_);
}

void RRTConnect::setStepSize(double eta)
{
  eta_ = eta;
  steering_->setStepSize(eta);
  if (fallback_)
    fallback_->setStepSize(eta);
  selection_->setStepSize(eta);
}

bool RRTConnect::solve(ConstRefVector start, ConstRefVector goal, double max_time)
{
  const StopWatch clock;
  reset();

  Vector start_root = start, goal_root = goal;
  if (!projection_.project(start_root, false) || !projection_.project(goal_root, false) || !problem_.isValid(start_root) ||
      !problem_.isValid(goal_root))
  {
    seconds_ = clock.elapsed();
    return false;
  }

  MotionTree start_tree(problem_, metric_, start_root), goal_tree(problem_, metric_, goal_root);
  MotionTree* a = &start_tree;
  MotionTree* b = &goal_tree;
  bool solved = false;
  while (!solved && stats_.iterations < max_iterations_ && clock.elapsed() < max_time)
  {
    ++stats_.iterations;
    sampleRandom(sample_);
    const int near = selection_->select(*a, sample_);
    if (near < 0)
    {
      ++stats_.declined;
      continue;
    }
    const int added = extend(*a, near, sample_);
    if (added < 0)
      continue;

    const Vector q_new = a->q(added);
    const int reached = connect(*b, q_new, clock, max_time);
    if (reached >= 0 && isEdgeValid(b->q(reached), q_new))
    {
      if (a == &start_tree)
        buildPath(start_tree, added, goal_tree, reached);
      else
        buildPath(start_tree, reached, goal_tree, added);
      solved = true;
    }
    else
    {
      std::swap(a, b);
    }
  }

  if (recording_)
  {
    trace_.start_edges = start_tree.edges();
    trace_.goal_edges = goal_tree.edges();
  }
  stats_.nodes = start_tree.size() + goal_tree.size();
  seconds_ = clock.elapsed();
  return solved;
}

void RRTConnect::reset()
{
  stats_ = PlannerStatistics();
  trace_ = SearchTrace();
  path_.clear();
  path_length_ = 0.0;
  rng_.seed(seed_);
  uniform_.reset();
}

void RRTConnect::sampleRandom(Vector& q)
{
  const Vector& lower = problem_.lowerBound();
  const Vector& upper = problem_.upperBound();
  q.resize(lower.size());
  for (int i = 0; i < q.size(); ++i)
    q[i] = lower[i] + uniform_(rng_) * (upper[i] - lower[i]);
}

int RRTConnect::extend(MotionTree& tree, int i, ConstRefVector target)
{
  const Chart* chart = tree.chart(i, steering_->usesBarrier());
  if (!chart)
    return -1;
  Outcome outcome = tryStep(*chart, tree.q(i), target, *steering_);
  if (outcome == Outcome::Invalid && fallback_)
  {
    const Chart* fallback_chart = tree.chart(i, fallback_->usesBarrier());
    if (fallback_chart)
    {
      ++stats_.fallbacks;
      outcome = tryStep(*fallback_chart, tree.q(i), target, *fallback_);
    }
  }
  if (outcome == Outcome::Invalid)
  {
    ++stats_.rejected;
    if (recording_)
      trace_.rejected.emplace_back(tree.q(i), next_);
  }
  return outcome == Outcome::Accepted ? tree.add(next_, i) : -1;
}

int RRTConnect::connect(MotionTree& tree, ConstRefVector target, const StopWatch& clock, double max_time)
{
  int current = selection_->select(tree, target);
  if (current < 0)
  {
    ++stats_.declined;
    return -1;
  }
  double distance = (tree.q(current) - target).norm();
  for (bool first = true; distance >= connection_tolerance_ && clock.elapsed() < max_time; first = false)
  {
    if (!first && !selection_->accepts(tree, current, target))
    {
      ++stats_.declined;
      return -1;
    }
    const int next = extend(tree, current, target);
    if (next < 0)
      return -1;
    current = next;
    const double next_distance = (tree.q(current) - target).norm();
    const bool closer = next_distance < distance;
    distance = next_distance;
    if (!closer)
      return -1;
  }
  return distance < connection_tolerance_ ? current : -1;
}

RRTConnect::Outcome RRTConnect::tryStep(const Chart& chart, ConstRefVector q, ConstRefVector target, SteeringBase& steering)
{
  if (!steering.step(chart, q, target, next_))
    return Outcome::NoDirection;
  if (!projection_.project(next_, steering.usesBarrier()))
    return Outcome::ProjectionFailed;
  if (!isValid(next_) || !isEdgeValid(q, next_))
    return Outcome::Invalid;
  return Outcome::Accepted;
}

bool RRTConnect::isValid(ConstRefVector q)
{
  ++stats_.checks;
  return problem_.isValid(q);
}

bool RRTConnect::isEdgeValid(ConstRefVector a, ConstRefVector b)
{
  const int pieces = static_cast<int>(std::ceil((b - a).norm() / edge_resolution_));
  for (int i = 1; i < pieces; ++i)
    if (!isValid(a + (b - a) * (static_cast<double>(i) / pieces)))
      return false;
  return true;
}

void RRTConnect::buildPath(const MotionTree& start_tree, int start_end, const MotionTree& goal_tree, int goal_end)
{
  path_ = start_tree.pathToRoot(start_end);
  std::reverse(path_.begin(), path_.end());
  const std::vector<Vector> backward = goal_tree.pathToRoot(goal_end);
  path_.insert(path_.end(), backward.begin(), backward.end());
  path_length_ = 0.0;
  for (std::size_t k = 0; k + 1 < path_.size(); ++k)
    path_length_ += (path_[k + 1] - path_[k]).norm();
}

}  // namespace rmrrt

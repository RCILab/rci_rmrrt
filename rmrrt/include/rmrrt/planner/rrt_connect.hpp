#pragma once

#include <memory>
#include <random>
#include <vector>

#include "rmrrt/planner/motion_tree.hpp"
#include "rmrrt/planner/newton_projection.hpp"
#include "rmrrt/planner/planner_statistics.hpp"
#include "rmrrt/selection/selection_base.hpp"
#include "rmrrt/steering/steering_base.hpp"
#include "rmrrt/util/stop_watch.hpp"

namespace rmrrt {

// Bidirectional RRT-Connect on an equality-constrained manifold. Its steering and node selection decide
// how the trees grow; RMRRT uses the barrier metric for both (rmrrt/planner/method.hpp). The defaults are
// RMRRT's. The inequalities are decided only by ProblemBase::isValid.
class RRTConnect
{
public:
  RRTConnect(const ProblemBase& problem, const RiemannianMetric& metric);

  bool solve(ConstRefVector start, ConstRefVector goal, double max_time);

  void setSteering(std::shared_ptr<SteeringBase> steering);
  // Tried once more from the same node when a step is rejected by the validity check; nullptr: none.
  void setFallback(std::shared_ptr<SteeringBase> steering);
  void setSelection(std::shared_ptr<SelectionBase> selection);
  void setStepSize(double eta);
  void setConnectionTolerance(double tolerance) { connection_tolerance_ = tolerance; }
  void setEdgeResolution(double resolution) { edge_resolution_ = resolution; }
  void setMaxIterations(std::size_t iterations) { max_iterations_ = iterations; }
  void setSeed(unsigned seed) { seed_ = seed; }
  void setRecording(bool recording) { recording_ = recording; }  // keep the trees and the rejected steps
  NewtonProjection& projection() { return projection_; }

  const std::vector<Vector>& path() const { return path_; }
  double pathLength() const { return path_length_; }
  double seconds() const { return seconds_; }
  const PlannerStatistics& statistics() const { return stats_; }
  const SearchTrace& trace() const { return trace_; }

private:
  enum class Outcome
  {
    Accepted,
    NoDirection,
    ProjectionFailed,
    Invalid
  };

  void reset();
  void sampleRandom(Vector& q);
  // One step from node i toward the target. The new node, or -1.
  int extend(MotionTree& tree, int i, ConstRefVector target);
  // Steps of the tree toward the target until it arrives, is blocked or gets no closer. The node that
  // arrived within the connection tolerance, or -1.
  int connect(MotionTree& tree, ConstRefVector target, const StopWatch& clock, double max_time);
  Outcome tryStep(const Chart& chart, ConstRefVector q, ConstRefVector target, SteeringBase& steering);
  bool isValid(ConstRefVector q);
  bool isEdgeValid(ConstRefVector a, ConstRefVector b);  // a is already known to be valid
  void buildPath(const MotionTree& start_tree, int start_end, const MotionTree& goal_tree, int goal_end);

  const ProblemBase& problem_;
  const RiemannianMetric& metric_;
  NewtonProjection projection_;
  std::shared_ptr<SteeringBase> steering_;
  std::shared_ptr<SteeringBase> fallback_;
  std::shared_ptr<SelectionBase> selection_;

  double eta_{0.2};
  double connection_tolerance_{0.2};
  double edge_resolution_{0.01};
  std::size_t max_iterations_{100000000};
  unsigned seed_{1};
  bool recording_{false};

  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  Vector sample_, next_;

  std::vector<Vector> path_;
  double path_length_{0.0};
  double seconds_{0.0};
  PlannerStatistics stats_;
  SearchTrace trace_;
};

}  // namespace rmrrt

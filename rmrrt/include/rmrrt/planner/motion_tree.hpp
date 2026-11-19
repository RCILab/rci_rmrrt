#pragma once

#include <memory>
#include <vector>

#include "rmrrt/metric/riemannian_metric.hpp"

namespace rmrrt {

// One tree of the search. Each node keeps its charts, built on first use.
class MotionTree
{
public:
  MotionTree(const ProblemBase& problem, const RiemannianMetric& metric, ConstRefVector root);

  int add(ConstRefVector q, int parent);
  const Vector& q(int i) const { return node(i).q; }
  int parent(int i) const { return node(i).parent; }
  std::size_t size() const { return nodes_.size(); }

  // The chart at node i, with or without the barrier; nullptr when it cannot be built.
  const Chart* chart(int i, bool use_barrier);
  // The configurations from node i back to the root.
  std::vector<Vector> pathToRoot(int i) const;
  // (node, parent) of every node but the root.
  std::vector<std::pair<Vector, Vector>> edges() const;

private:
  struct Node
  {
    Vector q;
    int parent{-1};
    std::unique_ptr<Chart> barrier_chart;
    std::unique_ptr<Chart> euclidean_chart;
  };

  const Node& node(int i) const { return nodes_[static_cast<std::size_t>(i)]; }

  const ProblemBase& problem_;
  const RiemannianMetric& metric_;
  std::vector<Node> nodes_;
};

}  // namespace rmrrt

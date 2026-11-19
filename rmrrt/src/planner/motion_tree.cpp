#include "rmrrt/planner/motion_tree.hpp"

namespace rmrrt {

MotionTree::MotionTree(const ProblemBase& problem, const RiemannianMetric& metric, ConstRefVector root)
  : problem_(problem), metric_(metric)
{
  add(root, -1);
}

int MotionTree::add(ConstRefVector q, int parent)
{
  nodes_.emplace_back();
  nodes_.back().q = q;
  nodes_.back().parent = parent;
  return static_cast<int>(nodes_.size()) - 1;
}

const Chart* MotionTree::chart(int i, bool use_barrier)
{
  Node& node = nodes_[static_cast<std::size_t>(i)];
  std::unique_ptr<Chart>& slot = use_barrier ? node.barrier_chart : node.euclidean_chart;
  if (!slot)
  {
    auto built = std::make_unique<Chart>();
    if (!metric_.computeChart(problem_, node.q, use_barrier, *built))
      return nullptr;
    slot = std::move(built);
  }
  return slot.get();
}

std::vector<Vector> MotionTree::pathToRoot(int i) const
{
  std::vector<Vector> states;
  for (; i >= 0; i = parent(i))
    states.push_back(q(i));
  return states;
}

std::vector<std::pair<Vector, Vector>> MotionTree::edges() const
{
  std::vector<std::pair<Vector, Vector>> list;
  for (std::size_t i = 1; i < nodes_.size(); ++i)
    list.emplace_back(nodes_[i].q, q(nodes_[i].parent));
  return list;
}

}  // namespace rmrrt

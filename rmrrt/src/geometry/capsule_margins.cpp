#include "rmrrt/geometry/capsule_margins.hpp"

#include <algorithm>
#include <map>
#include <utility>
#include <vector>

namespace rmrrt {

CapsuleMargins::CapsuleMargins(const KinematicsBase& kinematics, std::vector<Capsule> capsules,
                               std::vector<CapsulePair> self_pairs, std::vector<Box> obstacles, Vector lower, Vector upper)
  : kinematics_(kinematics)
  , capsules_(std::move(capsules))
  , self_pairs_(std::move(self_pairs))
  , obstacles_(std::move(obstacles))
  , lower_(std::move(lower))
  , upper_(std::move(upper))
{
  // Groups: one per (link, body), with the sphere that bounds its capsules.
  std::map<std::pair<int, int>, std::size_t> index;
  group_of_.resize(capsules_.size());
  for (std::size_t c = 0; c < capsules_.size(); ++c)
  {
    const auto key = std::make_pair(capsules_[c].link, capsules_[c].body);
    auto found = index.find(key);
    if (found == index.end())
    {
      found = index.emplace(key, groups_.size()).first;
      groups_.push_back(Group{capsules_[c].link, {}, Eigen::Vector3d::Zero(), 0.0});
    }
    group_of_[c] = found->second;
    groups_[found->second].capsules.push_back(c);
  }
  for (Group& group : groups_)
  {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const std::size_t c : group.capsules)
      sum += capsules_[c].a + capsules_[c].b;
    group.center = sum / (2.0 * static_cast<double>(group.capsules.size()));
    for (const std::size_t c : group.capsules)
      group.radius = std::max(group.radius, std::max((capsules_[c].a - group.center).norm(), (capsules_[c].b - group.center).norm()) +
                                                capsules_[c].radius);
  }
  std::map<std::pair<std::size_t, std::size_t>, std::size_t> pair_index;
  for (std::size_t p = 0; p < self_pairs_.size(); ++p)
  {
    std::size_t gi = group_of_[static_cast<std::size_t>(self_pairs_[p].first)], gj = group_of_[static_cast<std::size_t>(self_pairs_[p].second)];
    if (gi > gj)
      std::swap(gi, gj);
    auto found = pair_index.find({gi, gj});
    if (found == pair_index.end())
    {
      found = pair_index.emplace(std::make_pair(gi, gj), pair_groups_.size()).first;
      pair_groups_.emplace_back(gi, gj, std::vector<std::size_t>{});
    }
    std::get<2>(pair_groups_[found->second]).push_back(p);
  }
}

void CapsuleMargins::compute(ConstRefVector q, const Barrier& barrier, Margins& out) const
{
  const int n = static_cast<int>(q.size());

  if (barrier.enabled(Category::JointLimit))
  {
    const double width = barrier.width(Category::JointLimit);
    const auto add = [&](double h, int k, double sign) {
      if (h >= width)
        return;
      Margin margin;
      margin.h = h;
      margin.category = Category::JointLimit;
      margin.gradient = Vector::Zero(n);
      margin.gradient[k] = sign;
      out.push_back(std::move(margin));
    };
    for (int k = 0; k < n; ++k)
    {
      add(q[k] - lower_[k], k, 1.0);
      add(upper_[k] - q[k], k, -1.0);
    }
  }

  const bool use_obstacles = barrier.enabled(Category::Obstacle) && !obstacles_.empty();
  const bool use_self = barrier.enabled(Category::SelfCollision) && !self_pairs_.empty();
  if (!use_obstacles && !use_self)
    return;

  kinematics_.setConfiguration(q);
  std::vector<Eigen::Isometry3d> poses;
  std::vector<Eigen::Vector3d> a, b;
  poses.reserve(capsules_.size());
  a.reserve(capsules_.size());
  b.reserve(capsules_.size());
  for (const Capsule& capsule : capsules_)
  {
    poses.push_back(kinematics_.linkPose(capsule.link));
    a.push_back(poses.back() * capsule.a);
    b.push_back(poses.back() * capsule.b);
  }

  // At the closest point the distance is stationary along the segment, so the gradient is the
  // Jacobian of that point held fixed to its link: J_v - [p - o]x J_w, from the link Jacobian,
  // computed once per link.
  std::map<int, Eigen::Matrix<double, 6, Eigen::Dynamic>> link_jacobians;
  const auto jacobianAt = [&](std::size_t c, double t) -> Eigen::Matrix<double, 3, Eigen::Dynamic> {
    const int link = capsules_[c].link;
    auto found = link_jacobians.find(link);
    if (found == link_jacobians.end())
      found = link_jacobians.emplace(link, kinematics_.linkJacobian(link)).first;
    const Eigen::Vector3d r = a[c] + t * (b[c] - a[c]) - poses[c].translation();
    Eigen::Matrix3d cross;
    cross << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
    return found->second.topRows<3>() - cross * found->second.bottomRows<3>();
  };
  // Bounding sphere of each capsule, to skip the pairs that are surely farther apart than the width.
  std::vector<Eigen::Vector3d> center(capsules_.size());
  std::vector<double> reach(capsules_.size());
  for (std::size_t c = 0; c < capsules_.size(); ++c)
  {
    center[c] = 0.5 * (a[c] + b[c]);
    reach[c] = 0.5 * (b[c] - a[c]).norm() + capsules_[c].radius;
  }
  std::vector<Eigen::Vector3d> group_center(groups_.size());
  for (std::size_t g = 0; g < groups_.size(); ++g)
    group_center[g] = poses[groups_[g].capsules.front()] * groups_[g].center;

  if (use_obstacles)
  {
    const double width = barrier.width(Category::Obstacle);
    // Groups that may come within the width of each obstacle.
    std::vector<char> near(groups_.size() * obstacles_.size(), 0);
    for (std::size_t g = 0; g < groups_.size(); ++g)
      for (std::size_t o = 0; o < obstacles_.size(); ++o)
      {
        const SegmentBoxResult bound = segmentToBox(group_center[g], group_center[g], obstacles_[o], width + groups_[g].radius);
        near[g * obstacles_.size() + o] = !bound.exact || bound.distance - groups_[g].radius < width;
      }
    for (std::size_t c = 0; c < capsules_.size(); ++c)
    {
      if (capsules_[c].fixed)
        continue;
      const double radius = capsules_[c].radius;
      for (std::size_t o = 0; o < obstacles_.size(); ++o)
      {
        if (!near[group_of_[c] * obstacles_.size() + o])
          continue;
        const SegmentBoxResult result = segmentToBox(a[c], b[c], obstacles_[o], width + radius);
        if (!result.exact || result.distance - radius >= width)
          continue;
        Margin margin;
        margin.h = result.distance - radius;
        margin.category = Category::Obstacle;
        margin.gradient = jacobianAt(c, result.t).transpose() * result.normal;
        out.push_back(std::move(margin));
      }
    }
  }

  if (use_self)
  {
    const double width = barrier.width(Category::SelfCollision);
    // Pairs of groups whose spheres come within the width; their pairs in the order of self_pairs_.
    std::vector<std::size_t> candidates;
    for (const auto& [gi, gj, list] : pair_groups_)
      if ((group_center[gi] - group_center[gj]).norm() - groups_[gi].radius - groups_[gj].radius < width)
        candidates.insert(candidates.end(), list.begin(), list.end());
    std::sort(candidates.begin(), candidates.end());
    for (const std::size_t p : candidates)
    {
      const CapsulePair& pair = self_pairs_[p];
      const std::size_t i = static_cast<std::size_t>(pair.first), j = static_cast<std::size_t>(pair.second);
      if ((center[i] - center[j]).norm() - reach[i] - reach[j] >= width)
        continue;
      const SegmentSegmentResult result = segmentToSegment(a[i], b[i], a[j], b[j]);
      const double h = result.distance - capsules_[i].radius - capsules_[j].radius;
      if (h >= width)
        continue;
      Margin margin;
      margin.h = h;
      margin.category = Category::SelfCollision;
      margin.gradient = (jacobianAt(i, result.s) - jacobianAt(j, result.t)).transpose() * result.normal;
      out.push_back(std::move(margin));
    }
  }
}

}  // namespace rmrrt

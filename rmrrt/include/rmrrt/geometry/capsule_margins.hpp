#pragma once

#include <tuple>
#include <vector>

#include "rmrrt/geometry/distance.hpp"
#include "rmrrt/geometry/kinematics_base.hpp"
#include "rmrrt/metric/barrier.hpp"

namespace rmrrt {

// Segment a-b in the frame of a link, with a radius. a = b is a sphere.
struct Capsule
{
  int link{0};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  Eigen::Vector3d b{Eigen::Vector3d::Zero()};
  double radius{0.0};
  int body{-1};        // rigid part on the link, for the broadphase (-1: the link itself)
  bool fixed{false};   // on a link that never moves: its obstacle terms are constant and left out
};

struct CapsulePair
{
  int first{0};
  int second{0};
};

// Margins of a robot approximated by capsules:
//   obstacle        h = dist(capsule, box) - r              grad h = J(p*)^T n
//   self-collision  h = dist(seg_a, seg_b) - r_a - r_b      grad h = (J_a(p_a*) - J_b(p_b*))^T n
//   joint limit     h = q_k - q_min,  h = q_max - q_k       grad h = +e_k, -e_k
class CapsuleMargins
{
public:
  CapsuleMargins(const KinematicsBase& kinematics, std::vector<Capsule> capsules, std::vector<CapsulePair> self_pairs,
                 std::vector<Box> obstacles, Vector lower, Vector upper);

  // Appends the margins below the barrier width, in a fixed order: joint limits, obstacles, self pairs.
  void compute(ConstRefVector q, const Barrier& barrier, Margins& out) const;

private:
  // Capsules of one body and their bounding sphere in the link frame.
  struct Group
  {
    int link{0};
    std::vector<std::size_t> capsules;
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    double radius{0.0};
  };

  const KinematicsBase& kinematics_;
  std::vector<Capsule> capsules_;
  std::vector<CapsulePair> self_pairs_;
  std::vector<Group> groups_;
  std::vector<std::size_t> group_of_;  // capsule -> group
  // Self pairs by pair of groups: (group, group, pair indices).
  std::vector<std::tuple<std::size_t, std::size_t, std::vector<std::size_t>>> pair_groups_;
  std::vector<Box> obstacles_;
  Vector lower_, upper_;
};

}  // namespace rmrrt

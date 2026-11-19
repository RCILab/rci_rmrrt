#pragma once

#include <vector>

#include "rmrrt/robot/moveit_kinematics.hpp"
#include "rmrrt/robot/scenario.hpp"

namespace rmrrt {
namespace robot {

// The equality constraint g(q) = 0 of a scenario:
//   closed chain   the tool frames keep the relative poses they have at the start (6 per extra tool)
//   direction      a tool axis stays along a world axis (2)
class GraspConstraint
{
public:
  GraspConstraint(MoveitKinematics& kinematics, const ScenarioSpec& spec, ConstRefVector start);

  int coDim() const { return co_dim_; }
  void function(ConstRefVector q, RefVector g) const;
  void jacobian(ConstRefVector q, RefMatrix J) const;

private:
  // The body Jacobian of a tool frame: its twist in its own frame.
  Matrix bodyJacobian(int tip, const Eigen::Isometry3d& pose) const;

  const MoveitKinematics& kinematics_;
  std::vector<int> tips_;
  std::vector<Eigen::Isometry3d> grasps_;  // T_0^-1 T_k at the start
  int direction_link_{-1};
  Eigen::Vector3d direction_local_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d direction_e1_{Eigen::Vector3d::UnitX()}, direction_e2_{Eigen::Vector3d::UnitY()};  // normal to the world axis
  int co_dim_{0};
};

}  // namespace robot
}  // namespace rmrrt

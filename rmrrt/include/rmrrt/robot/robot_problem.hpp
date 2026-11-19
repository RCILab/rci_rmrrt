#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rmrrt/core/problem_base.hpp"
#include "rmrrt/robot/grasp_constraint.hpp"
#include "rmrrt/robot/moveit_kinematics.hpp"
#include "rmrrt/robot/robot_scene.hpp"

namespace rmrrt {
namespace robot {

// One query of a scenario as a planning problem: MoveIt gives the kinematics and the validity check, the
// capsules of the robot give the margins of the metric, and the grasp at the start gives g(q) = 0.
class RobotProblem : public ProblemBase
{
public:
  // scene_file: the boxes of this query (empty: the scenario's obstacles only).
  RobotProblem(const moveit::core::RobotModelConstPtr& model, const ScenarioSpec& spec, const std::string& scene_file,
               ConstRefVector start, ConstRefVector goal);

  int dim() const override { return kinematics_.dim(); }
  int coDim() const override { return constraint_->coDim(); }
  void function(ConstRefVector q, RefVector g) const override { constraint_->function(q, g); }
  void jacobian(ConstRefVector q, RefMatrix J) const override { constraint_->jacobian(q, J); }
  const Vector& lowerBound() const override { return lower_; }
  const Vector& upperBound() const override { return upper_; }
  void computeMargins(ConstRefVector q, const Barrier& barrier, Margins& margins) const override;
  bool isValid(ConstRefVector q) const override;

  // Why q is invalid, for messages.
  std::string describeInvalid(ConstRefVector q) const;
  // The robot at q with the object attached. Valid until the next call.
  const moveit::core::RobotState& state(ConstRefVector q) const;
  const RobotScene& scene() const { return scene_; }

private:
  void setJointLimits();
  void setCapsules(const ScenarioSpec& spec, ConstRefVector start, ConstRefVector goal);

  moveit::core::RobotModelConstPtr model_;
  MoveitKinematics kinematics_;
  RobotScene scene_;
  std::vector<const moveit::core::JointModelGroup*> groups_;
  mutable moveit::core::RobotState state_;
  mutable std::vector<double> segment_;
  Vector lower_, upper_;
  std::unique_ptr<CapsuleMargins> margins_;
  std::unique_ptr<GraspConstraint> constraint_;
};

}  // namespace robot
}  // namespace rmrrt

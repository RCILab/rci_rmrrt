#include "rmrrt/robot/robot_problem.hpp"

#include <algorithm>
#include <cmath>

#include "rmrrt/robot/robot_capsules.hpp"

namespace rmrrt {
namespace robot {

RobotProblem::RobotProblem(const moveit::core::RobotModelConstPtr& model, const ScenarioSpec& spec, const std::string& scene_file,
                           ConstRefVector start, ConstRefVector goal)
  : model_(model), kinematics_(model, spec.groups), scene_(model, spec, scene_file), state_(scene_.defaultState())
{
  for (const std::string& name : spec.groups)
    groups_.push_back(model->getJointModelGroup(name));
  setJointLimits();
  setCapsules(spec, start, goal);
  constraint_ = std::make_unique<GraspConstraint>(kinematics_, spec, start);
}

void RobotProblem::computeMargins(ConstRefVector q, const Barrier& barrier, Margins& margins) const
{
  margins_->compute(q, barrier, margins);
}

bool RobotProblem::isValid(ConstRefVector q) const
{
  const auto& s = state(q);
  for (const auto* group : groups_)
    if (!s.satisfiesBounds(group))
      return false;
  return scene_.isValid(s);
}

std::string RobotProblem::describeInvalid(ConstRefVector q) const
{
  const auto& s = state(q);
  std::string reason;
  for (const auto* group : groups_)
    if (!s.satisfiesBounds(group))
      reason += "joint limit in " + group->getName() + " ";
  return reason + scene_.describeContact(s);
}

const moveit::core::RobotState& RobotProblem::state(ConstRefVector q) const
{
  int offset = 0;
  for (const auto* group : groups_)
  {
    const int dof = static_cast<int>(group->getVariableCount());
    segment_.assign(q.data() + offset, q.data() + offset + dof);
    state_.setJointGroupPositions(group, segment_);
    offset += dof;
  }
  state_.update();
  return state_;
}

// The joint limits are the sampling box; an unbounded joint samples [-pi, pi].
void RobotProblem::setJointLimits()
{
  lower_.resize(dim());
  upper_.resize(dim());
  int index = 0;
  for (const auto* group : groups_)
    for (const std::string& variable : group->getVariableNames())
    {
      const auto& bounds = model_->getVariableBounds(variable);
      lower_[index] = bounds.position_bounded_ ? bounds.min_position_ : -M_PI;
      upper_[index] = bounds.position_bounded_ ? bounds.max_position_ : M_PI;
      ++index;
    }
}

// Capsules of the robot and the carried object, and the pairs of them checked for self-collision: on
// different rigid bodies, allowed to collide by the scene, and not overlapping at both the start and the
// goal (such a pair overlaps by construction and carries no information).
void RobotProblem::setCapsules(const ScenarioSpec& spec, ConstRefVector start, ConstRefVector goal)
{
  std::vector<Capsule> capsules;
  std::vector<std::string> owners;
  for (auto& [link, capsule] : linkCapsules(*model_, spec.world_box_links, spec.link_capsules))
  {
    capsule.link = kinematics_.linkIndex(link);
    capsule.fixed = !kinematics_.moves(capsule.link);
    capsules.push_back(capsule);
    owners.push_back(link);
  }
  const std::size_t robot_capsules = capsules.size();
  if (spec.has_carried)
    for (Capsule capsule : carriedCapsules(spec.carried))
    {
      capsule.link = kinematics_.linkIndex(spec.carried.link);
      capsule.body = 1;  // apart from the capsules of the link that holds it
      capsule.fixed = !kinematics_.moves(capsule.link);
      capsules.push_back(capsule);
      owners.push_back(spec.carried.id);
    }

  const RigidBodies rigid(model_, spec.groups);
  std::vector<std::string> bodies;
  for (std::size_t c = 0; c < capsules.size(); ++c)
    bodies.push_back(rigid.root(c < robot_capsules ? owners[c] : spec.carried.link));
  const auto gap = [&](ConstRefVector q, std::size_t i, std::size_t j) {
    kinematics_.setConfiguration(q);
    const Eigen::Isometry3d pi = kinematics_.linkPose(capsules[i].link), pj = kinematics_.linkPose(capsules[j].link);
    return segmentToSegment(pi * capsules[i].a, pi * capsules[i].b, pj * capsules[j].a, pj * capsules[j].b).distance -
           capsules[i].radius - capsules[j].radius;
  };
  const auto& allowed = scene_.allowedCollisions();
  const auto& ignored = spec.carried.ignore_links;
  std::vector<CapsulePair> pairs;
  for (std::size_t i = 0; i < capsules.size(); ++i)
    for (std::size_t j = i + 1; j < capsules.size(); ++j)
    {
      if (owners[i] == owners[j] || bodies[i] == bodies[j])
        continue;
      if (!kinematics_.moves(capsules[i].link) && !kinematics_.moves(capsules[j].link))
        continue;
      collision_detection::AllowedCollision::Type type;
      if (j < robot_capsules && allowed.getEntry(owners[i], owners[j], type) && type == collision_detection::AllowedCollision::ALWAYS)
        continue;
      if (j >= robot_capsules && std::find(ignored.begin(), ignored.end(), owners[i]) != ignored.end())
        continue;
      if (gap(start, i, j) < 0.0 && gap(goal, i, j) < 0.0)
        continue;
      pairs.push_back({static_cast<int>(i), static_cast<int>(j)});
    }

  margins_ = std::make_unique<CapsuleMargins>(kinematics_, std::move(capsules), std::move(pairs), scene_.obstacles(), lower_, upper_);
}

}  // namespace robot
}  // namespace rmrrt

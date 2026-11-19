#include "rmrrt/robot/moveit_kinematics.hpp"

#include <stdexcept>

namespace rmrrt {
namespace robot {

MoveitKinematics::MoveitKinematics(const moveit::core::RobotModelConstPtr& model, const std::vector<std::string>& groups)
  : model_(model), state_(model)
{
  state_.setToDefaultValues();
  for (const std::string& name : groups)
  {
    Arm arm;
    arm.group = model->getJointModelGroup(name);
    if (!arm.group)
      throw std::runtime_error("Unknown group " + name);
    arm.root = arm.group->getJointModels().front()->getParentLinkModel();
    arm.offset = dim_;
    arm.dof = static_cast<int>(arm.group->getVariableCount());
    dim_ += arm.dof;
    arms_.push_back(arm);
  }
}

int MoveitKinematics::linkIndex(const std::string& name)
{
  const auto found = indices_.find(name);
  if (found != indices_.end())
    return found->second;
  Link link;
  link.model = model_->getLinkModel(name);
  if (!link.model)
    throw std::runtime_error("Unknown link " + name);
  for (std::size_t a = 0; a < arms_.size(); ++a)
    if (arms_[a].group->isLinkUpdated(name))
      link.arm = static_cast<int>(a);
  links_.push_back(link);
  return indices_[name] = static_cast<int>(links_.size()) - 1;
}

void MoveitKinematics::setConfiguration(ConstRefVector q) const
{
  for (const Arm& arm : arms_)
  {
    segment_.assign(q.data() + arm.offset, q.data() + arm.offset + arm.dof);
    state_.setJointGroupPositions(arm.group, segment_);
  }
  state_.update();
}

Eigen::Isometry3d MoveitKinematics::linkPose(int link) const
{
  return state_.getGlobalLinkTransform(links_[static_cast<std::size_t>(link)].model);
}

Eigen::Matrix<double, 3, Eigen::Dynamic> MoveitKinematics::pointJacobian(int link, const Eigen::Vector3d& point) const
{
  Eigen::Matrix<double, 3, Eigen::Dynamic> J = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, dim_);
  const Link& entry = links_[static_cast<std::size_t>(link)];
  if (entry.arm < 0)
    return J;
  const Arm& arm = arms_[static_cast<std::size_t>(entry.arm)];
  if (!state_.getJacobian(arm.group, entry.model, point, jacobian_))
    throw std::runtime_error("No Jacobian for " + entry.model->getName());
  J.middleCols(arm.offset, arm.dof) = rootRotation(arm) * jacobian_.topRows<3>();
  return J;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> MoveitKinematics::linkJacobian(int link) const
{
  Eigen::Matrix<double, 6, Eigen::Dynamic> J = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, dim_);
  const Link& entry = links_[static_cast<std::size_t>(link)];
  if (entry.arm < 0)
    return J;
  const Arm& arm = arms_[static_cast<std::size_t>(entry.arm)];
  if (!state_.getJacobian(arm.group, entry.model, Eigen::Vector3d::Zero(), jacobian_))
    throw std::runtime_error("No Jacobian for " + entry.model->getName());
  const Eigen::Matrix3d R = rootRotation(arm);
  J.block(0, arm.offset, 3, arm.dof) = R * jacobian_.topRows<3>();
  J.block(3, arm.offset, 3, arm.dof) = R * jacobian_.bottomRows<3>();
  return J;
}

Eigen::Matrix3d MoveitKinematics::rootRotation(const Arm& arm) const
{
  return arm.root ? Eigen::Matrix3d(state_.getGlobalLinkTransform(arm.root).linear()) : Eigen::Matrix3d::Identity();
}

}  // namespace robot
}  // namespace rmrrt

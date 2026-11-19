#pragma once

#include <map>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "rmrrt/geometry/kinematics_base.hpp"

namespace rmrrt {
namespace robot {

// Forward kinematics of the planned groups, through MoveIt. q is the joints of the groups in order.
class MoveitKinematics : public KinematicsBase
{
public:
  MoveitKinematics(const moveit::core::RobotModelConstPtr& model, const std::vector<std::string>& groups);

  int dim() const override { return dim_; }
  void setConfiguration(ConstRefVector q) const override;
  Eigen::Isometry3d linkPose(int link) const override;
  Eigen::Matrix<double, 3, Eigen::Dynamic> pointJacobian(int link, const Eigen::Vector3d& point) const override;
  // Velocity of the link origin and angular velocity, in the world frame.
  Eigen::Matrix<double, 6, Eigen::Dynamic> linkJacobian(int link) const override;

  // The index of a link for linkPose and the Jacobians.
  int linkIndex(const std::string& name);
  // Whether a link moves with q.
  bool moves(int link) const { return links_[static_cast<std::size_t>(link)].arm >= 0; }

private:
  struct Arm
  {
    const moveit::core::JointModelGroup* group{nullptr};
    const moveit::core::LinkModel* root{nullptr};
    int offset{0};
    int dof{0};
  };

  struct Link
  {
    const moveit::core::LinkModel* model{nullptr};
    int arm{-1};  // -1: moves with no group
  };

  // MoveIt's Jacobian of a link, turned from the group's root frame into the world frame.
  Eigen::Matrix3d rootRotation(const Arm& arm) const;

  moveit::core::RobotModelConstPtr model_;
  std::vector<Arm> arms_;
  std::vector<Link> links_;
  std::map<std::string, int> indices_;
  int dim_{0};
  mutable moveit::core::RobotState state_;
  mutable Eigen::MatrixXd jacobian_;
  mutable std::vector<double> segment_;
};

}  // namespace robot
}  // namespace rmrrt

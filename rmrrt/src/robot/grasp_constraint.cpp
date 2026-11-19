#include "rmrrt/robot/grasp_constraint.hpp"

namespace rmrrt {
namespace robot {

GraspConstraint::GraspConstraint(MoveitKinematics& kinematics, const ScenarioSpec& spec, ConstRefVector start)
  : kinematics_(kinematics)
{
  for (const std::string& tip : spec.tips)
    tips_.push_back(kinematics.linkIndex(tip));
  co_dim_ = 6 * (static_cast<int>(tips_.size()) - 1);
  if (spec.has_direction)
  {
    co_dim_ += 2;
    direction_link_ = kinematics.linkIndex(spec.direction.link);
    direction_local_ = spec.direction.local;
    direction_e1_ = spec.direction.world.unitOrthogonal();
    direction_e2_ = spec.direction.world.cross(direction_e1_);
  }
  kinematics.setConfiguration(start);
  const Eigen::Isometry3d T0 = kinematics.linkPose(tips_[0]);
  for (std::size_t k = 1; k < tips_.size(); ++k)
    grasps_.push_back(T0.inverse() * kinematics.linkPose(tips_[k]));
}

// e = C^-1 T_0^-1 T_k with C the grasp at the start, g = [translation(e); log(rotation(e))].
void GraspConstraint::function(ConstRefVector q, RefVector g) const
{
  kinematics_.setConfiguration(q);
  const Eigen::Isometry3d T0_inverse = kinematics_.linkPose(tips_[0]).inverse();
  for (std::size_t k = 1; k < tips_.size(); ++k)
  {
    const Eigen::Isometry3d e = grasps_[k - 1].inverse() * T0_inverse * kinematics_.linkPose(tips_[k]);
    const Eigen::AngleAxisd rotation(e.linear());
    const int row = 6 * (static_cast<int>(k) - 1);
    g.segment<3>(row) = e.translation();
    g.segment<3>(row + 3) = rotation.angle() * rotation.axis();
  }
  if (direction_link_ >= 0)
  {
    const Eigen::Vector3d u = kinematics_.linkPose(direction_link_).linear() * direction_local_;
    g[co_dim_ - 2] = direction_e1_.dot(u);
    g[co_dim_ - 1] = direction_e2_.dot(u);
  }
}

// To first order around e = I, with xi the body twist of a tool frame:
//   dg/dq = xi_k - Ad(A^-1) xi_0,  A = T_0^-1 T_k.
void GraspConstraint::jacobian(ConstRefVector q, RefMatrix J) const
{
  J.setZero();
  kinematics_.setConfiguration(q);
  const Eigen::Isometry3d T0 = kinematics_.linkPose(tips_[0]);
  const Matrix body0 = bodyJacobian(tips_[0], T0);
  for (std::size_t k = 1; k < tips_.size(); ++k)
  {
    const Eigen::Isometry3d Tk = kinematics_.linkPose(tips_[k]);
    const Matrix bodyk = bodyJacobian(tips_[k], Tk);
    const Eigen::Isometry3d A = T0.inverse() * Tk;
    const Eigen::Matrix3d Rt = A.linear().transpose();
    const Eigen::Vector3d p = A.translation();
    Eigen::Matrix3d hat;
    hat << 0.0, -p.z(), p.y(), p.z(), 0.0, -p.x(), -p.y(), p.x(), 0.0;
    Eigen::Matrix<double, 6, 6> adjoint = Eigen::Matrix<double, 6, 6>::Zero();
    adjoint.topLeftCorner<3, 3>() = Rt;
    adjoint.topRightCorner<3, 3>() = -Rt * hat;
    adjoint.bottomRightCorner<3, 3>() = Rt;
    J.middleRows<6>(6 * (static_cast<int>(k) - 1)) = bodyk - adjoint * body0;
  }
  // du/dt = omega x u, so d(e . u) = (u x e)^T omega.
  if (direction_link_ >= 0)
  {
    const Eigen::Vector3d u = kinematics_.linkPose(direction_link_).linear() * direction_local_;
    const auto twist = kinematics_.linkJacobian(direction_link_);
    J.row(co_dim_ - 2) = u.cross(direction_e1_).transpose() * twist.bottomRows<3>();
    J.row(co_dim_ - 1) = u.cross(direction_e2_).transpose() * twist.bottomRows<3>();
  }
}

Matrix GraspConstraint::bodyJacobian(int tip, const Eigen::Isometry3d& pose) const
{
  const auto twist = kinematics_.linkJacobian(tip);
  Matrix body(6, kinematics_.dim());
  body.topRows<3>() = pose.linear().transpose() * twist.topRows<3>();
  body.bottomRows<3>() = pose.linear().transpose() * twist.bottomRows<3>();
  return body;
}

}  // namespace robot
}  // namespace rmrrt

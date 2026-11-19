#pragma once

#include <Eigen/Geometry>

#include "rmrrt/core/types.hpp"

namespace rmrrt {

// Forward kinematics of a robot in the world frame, for the capsule margins.
class KinematicsBase
{
public:
  virtual ~KinematicsBase() = default;

  virtual int dim() const = 0;
  virtual void setConfiguration(ConstRefVector q) const = 0;
  virtual Eigen::Isometry3d linkPose(int link) const = 0;
  // Linear velocity Jacobian of a point fixed to a link, the point in the link frame.
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> pointJacobian(int link, const Eigen::Vector3d& point) const = 0;

  // Linear velocity of the link origin (top) and angular velocity (bottom), both in the world frame.
  // The default builds it from point Jacobians: with r_k the world link axes and D_k = J(r_k) - J(0),
  // D_k = -[r_k]x J_w, and sum_k [r_k]x D_k = 2 J_w.
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic> linkJacobian(int link) const
  {
    const Eigen::Matrix<double, 3, Eigen::Dynamic> origin = pointJacobian(link, Eigen::Vector3d::Zero());
    const Eigen::Matrix3d R = linkPose(link).linear();
    Eigen::Matrix<double, 3, Eigen::Dynamic> angular = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, origin.cols());
    for (int k = 0; k < 3; ++k)
    {
      const Eigen::Vector3d r = R.col(k);
      Eigen::Matrix3d cross;
      cross << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
      angular += cross * (pointJacobian(link, Eigen::Vector3d::Unit(k)) - origin);
    }
    Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, origin.cols());
    J.topRows<3>() = origin;
    J.bottomRows<3>() = 0.5 * angular;
    return J;
  }
};

}  // namespace rmrrt

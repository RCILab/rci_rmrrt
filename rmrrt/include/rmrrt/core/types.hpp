#pragma once

#include <Eigen/Dense>

namespace rmrrt {

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
using RefVector = Eigen::Ref<Vector>;
using RefMatrix = Eigen::Ref<Matrix>;
using ConstRefVector = Eigen::Ref<const Vector>;
using ConstRefMatrix = Eigen::Ref<const Matrix>;

}  // namespace rmrrt

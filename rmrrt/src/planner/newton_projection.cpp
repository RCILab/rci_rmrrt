#include "rmrrt/planner/newton_projection.hpp"

namespace rmrrt {

bool NewtonProjection::project(RefVector q, bool use_barrier) const
{
  const int m = problem_.coDim();
  if (m == 0)
    return true;
  Vector g(m);
  Matrix J(m, problem_.dim());
  Matrix G;
  for (int iteration = 0; iteration <= max_iterations_; ++iteration)
  {
    problem_.function(q, g);
    if (!g.allFinite())
      return false;
    if (g.norm() < tau_)
      return true;
    if (iteration == max_iterations_)
      return false;

    problem_.jacobian(q, J);
    metric_.computeG(problem_, q, use_barrier, G);
    const Eigen::LLT<Matrix> llt(G);
    if (llt.info() != Eigen::Success)
      return false;
    const Matrix G_inv_Jt = llt.solve(J.transpose());
    const Eigen::LDLT<Matrix> schur(J * G_inv_Jt);
    if (schur.info() != Eigen::Success)
      return false;
    Vector update = -G_inv_Jt * schur.solve(g);
    const double norm = update.norm();
    if (norm > max_step_)
      update *= max_step_ / norm;
    q += update;
  }
  return false;
}

}  // namespace rmrrt

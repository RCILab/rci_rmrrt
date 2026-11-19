#include "rmrrt/metric/riemannian_metric.hpp"

namespace rmrrt {

bool RiemannianMetric::computeChart(const ProblemBase& problem, ConstRefVector q, bool use_barrier, Chart& chart) const
{
  const int n = problem.dim();
  chart.G = lambda_ * Matrix::Identity(n, n);
  chart.force = Vector::Zero(n);
  if (use_barrier)
  {
    Margins margins;
    problem.computeMargins(q, barrier_, margins);
    addWeights(margins, chart.G);
    addPush(margins, chart.force);
  }

  const Eigen::LLT<Matrix> llt(chart.G);
  if (llt.info() != Eigen::Success)
    return false;
  const Matrix G_inv = llt.solve(Matrix::Identity(n, n));
  if (problem.coDim() == 0)
  {
    chart.P = Matrix::Identity(n, n);
    chart.Q = G_inv;
    return true;
  }

  const Matrix J = problem.J(q);
  const Matrix G_inv_Jt = G_inv * J.transpose();
  const Eigen::LDLT<Matrix> schur(J * G_inv_Jt);
  if (schur.info() != Eigen::Success || schur.rcond() < 1e-14)
    return false;
  chart.P = Matrix::Identity(n, n) - G_inv_Jt * schur.solve(J);
  chart.Q = G_inv - G_inv_Jt * schur.solve(G_inv_Jt.transpose());
  return true;
}

void RiemannianMetric::computeG(const ProblemBase& problem, ConstRefVector q, bool use_barrier, Matrix& G) const
{
  const int n = problem.dim();
  G = lambda_ * Matrix::Identity(n, n);
  if (!use_barrier)
    return;
  Margins margins;
  problem.computeMargins(q, barrier_, margins);
  addWeights(margins, G);
}

void RiemannianMetric::addWeights(const Margins& margins, Matrix& G) const
{
  for (const Margin& margin : margins)
  {
    const double w = barrier_.weight(margin);
    if (w > 0.0)
      G.noalias() += w * (margin.gradient * margin.gradient.transpose());
  }
}

void RiemannianMetric::addPush(const Margins& margins, Vector& force) const
{
  for (const Margin& margin : margins)
  {
    const double c = barrier_.push(margin);
    if (c > 0.0)
      force.noalias() += c * margin.gradient;
  }
}

}  // namespace rmrrt

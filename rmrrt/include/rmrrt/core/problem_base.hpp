#pragma once

#include "rmrrt/core/margin.hpp"
#include "rmrrt/metric/barrier.hpp"

namespace rmrrt {

// A planning problem: the equality constraint g(q) = 0, the sampling box and the inequalities.
// The planner sees a robot only through this interface.
class ProblemBase
{
public:
  virtual ~ProblemBase() = default;

  virtual int dim() const = 0;    // n
  virtual int coDim() const = 0;  // m, the number of equality constraints
  virtual void function(ConstRefVector q, RefVector g) const = 0;
  virtual void jacobian(ConstRefVector q, RefMatrix J) const = 0;  // m x n
  virtual const Vector& lowerBound() const = 0;
  virtual const Vector& upperBound() const = 0;
  // Appends the inequalities that are within the barrier width of their bound.
  virtual void computeMargins(ConstRefVector q, const Barrier& barrier, Margins& margins) const = 0;
  // Collision and joint limits: the only place where the inequalities are decided.
  virtual bool isValid(ConstRefVector q) const = 0;

  Vector g(ConstRefVector q) const
  {
    Vector out(coDim());
    function(q, out);
    return out;
  }

  Matrix J(ConstRefVector q) const
  {
    Matrix out(coDim(), dim());
    jacobian(q, out);
    return out;
  }
};

}  // namespace rmrrt

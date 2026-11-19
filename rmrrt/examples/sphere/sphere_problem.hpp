#pragma once

// The sphere example: the unit sphere in R^3 (one equality constraint) with balls as obstacles, which cut
// holes out of it. No robot needed.

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "rmrrt/core/problem_base.hpp"
#include "rmrrt/planner/method.hpp"

namespace rmrrt {
namespace sphere {

class SphereProblem : public ProblemBase
{
public:
  SphereProblem(std::vector<Eigen::Vector3d> centers, std::vector<double> radii)
    : centers_(std::move(centers)), radii_(std::move(radii)), lower_(Vector::Constant(3, -1.2)), upper_(Vector::Constant(3, 1.2))
  {
  }

  int dim() const override { return 3; }
  int coDim() const override { return 1; }
  void function(ConstRefVector q, RefVector g) const override { g[0] = 0.5 * (q.squaredNorm() - 1.0); }
  void jacobian(ConstRefVector q, RefMatrix J) const override { J = q.transpose(); }
  const Vector& lowerBound() const override { return lower_; }
  const Vector& upperBound() const override { return upper_; }

  void computeMargins(ConstRefVector q, const Barrier& barrier, Margins& margins) const override
  {
    for (std::size_t i = 0; i < centers_.size(); ++i)
    {
      const Eigen::Vector3d offset = q - centers_[i];
      const double h = offset.norm() - radii_[i];
      if (h >= barrier.width(Category::Obstacle))
        continue;
      Margin margin;
      margin.h = h;
      margin.category = Category::Obstacle;
      margin.gradient = offset.normalized();
      margins.push_back(margin);
    }
  }

  bool isValid(ConstRefVector q) const override
  {
    for (std::size_t i = 0; i < centers_.size(); ++i)
      if ((q - centers_[i]).norm() <= radii_[i])
        return false;
    return true;
  }

private:
  std::vector<Eigen::Vector3d> centers_;
  std::vector<double> radii_;
  Vector lower_, upper_;
};

struct SphereInstance
{
  std::vector<Eigen::Vector3d> centers;
  std::vector<double> radii;
  Eigen::Vector3d start, goal;
};

inline Eigen::Vector3d onSphere(double latitude, double longitude)
{
  return Eigen::Vector3d(std::cos(latitude) * std::cos(longitude), std::cos(latitude) * std::sin(longitude), std::sin(latitude));
}

// A wall of overlapping balls along a parallel, with an opening of half-width h at longitude `gap`.
inline void addRing(SphereInstance& instance, double latitude, double gap, double h)
{
  const double step = 0.08, radius = 0.07;
  const int count = static_cast<int>(2.0 * M_PI * std::cos(latitude) / step);
  for (int k = 0; k < count; ++k)
  {
    const double longitude = 2.0 * M_PI * k / count;
    if (std::abs(std::remainder(longitude - gap, 2.0 * M_PI)) < h)
      continue;
    instance.centers.push_back(onSphere(latitude, longitude));
    instance.radii.push_back(radius);
  }
}

// Instance s of a layout: open | clutter | gate | double_gate. h is the half-width of the openings.
inline SphereInstance makeInstance(const std::string& layout, double h, int s)
{
  std::mt19937 rng(1000 + s);
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  const auto randomUnit = [&] { return Eigen::Vector3d(normal(rng), normal(rng), normal(rng)).normalized(); };
  const auto nearPole = [&](double sign, double spread) {
    return Eigen::Vector3d(Eigen::Vector3d(0.0, 0.0, sign) + spread * Eigen::Vector3d(normal(rng), normal(rng), normal(rng))).normalized();
  };
  const auto clear = [](const SphereInstance& instance, const Eigen::Vector3d& q) {
    for (std::size_t i = 0; i < instance.centers.size(); ++i)
      if ((q - instance.centers[i]).norm() <= instance.radii[i])
        return false;
    return true;
  };
  while (true)
  {
    SphereInstance instance;
    if (layout == "open" || layout == "clutter")
    {
      const bool open = layout == "open";
      const int count = open ? 6 : 45;
      const double low = open ? 0.15 : 0.10, high = open ? 0.25 : 0.20;
      for (int i = 0; i < count; ++i)
      {
        instance.centers.push_back(randomUnit());
        instance.radii.push_back(low + (high - low) * uniform(rng));
      }
      instance.start = nearPole(1.0, 0.25);
      instance.goal = nearPole(-1.0, 0.25);
    }
    else if (layout == "gate" || layout == "double_gate")
    {
      const double gap = 2.0 * M_PI * uniform(rng);
      if (layout == "gate")
      {
        addRing(instance, 0.0, gap, h);
      }
      else
      {
        addRing(instance, 0.45, gap, h);
        addRing(instance, -0.45, gap + M_PI, h);
      }
      instance.start = nearPole(1.0, 0.15);
      instance.goal = nearPole(-1.0, 0.15);
    }
    else
    {
      throw std::invalid_argument("Unknown layout " + layout + " (open | clutter | gate | double_gate)");
    }
    if (clear(instance, instance.start) && clear(instance, instance.goal))
      return instance;
  }
}

// The planner settings of the sphere: a shorter step for a smaller space.
inline void setUp(RRTConnect& planner, const Method& method, int s)
{
  configure(planner, method, 0.5);
  planner.setStepSize(0.08);
  planner.setConnectionTolerance(0.08);
  planner.setEdgeResolution(0.01);
  planner.projection().setTolerance(1e-3);
  planner.setMaxIterations(20000);
  planner.setSeed(5000 + static_cast<unsigned>(s));
}

}  // namespace sphere
}  // namespace rmrrt

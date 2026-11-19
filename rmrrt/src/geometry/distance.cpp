#include "rmrrt/geometry/distance.hpp"

#include <algorithm>
#include <cmath>

namespace rmrrt {

PointBoxResult pointToBox(const Eigen::Vector3d& point, const Box& box)
{
  const Eigen::Matrix3d R = box.pose.linear();
  const Eigen::Vector3d local = R.transpose() * (point - box.pose.translation());
  const Eigen::Vector3d excess = local.cwiseAbs() - box.half;
  const Eigen::Vector3d outside = excess.cwiseMax(0.0);
  const Eigen::Vector3d sign(local.x() >= 0.0 ? 1.0 : -1.0, local.y() >= 0.0 ? 1.0 : -1.0, local.z() >= 0.0 ? 1.0 : -1.0);

  PointBoxResult result;
  const double norm = outside.norm();
  Eigen::Vector3d local_normal = Eigen::Vector3d::Zero();
  if (norm > 1e-12)
  {
    result.distance = norm;
    local_normal = sign.cwiseProduct(outside) / norm;
  }
  else
  {
    int axis = 0;
    result.distance = excess.maxCoeff(&axis);
    local_normal[axis] = sign[axis];
  }
  result.normal = R * local_normal;
  return result;
}

SegmentBoxResult segmentToBox(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Box& box, double cutoff)
{
  SegmentBoxResult result;
  const Eigen::Vector3d d = b - a;
  const double length = d.norm();

  // The signed distance is 1-Lipschitz, which bounds it from below along the segment.
  const PointBoxResult middle = pointToBox(a + 0.5 * d, box);
  const double lower = middle.distance - 0.5 * length;
  if (lower >= cutoff)
  {
    result.distance = lower;
    result.exact = false;
    result.normal = middle.normal;
    return result;
  }

  if (length < 1e-12)
  {
    result.distance = middle.distance;
    result.t = 0.5;
    result.normal = middle.normal;
    return result;
  }

  const double ratio = 0.5 * (std::sqrt(5.0) - 1.0);
  double lo = 0.0, hi = 1.0;
  double x1 = hi - ratio * (hi - lo), x2 = lo + ratio * (hi - lo);
  double f1 = pointToBox(a + x1 * d, box).distance, f2 = pointToBox(a + x2 * d, box).distance;
  while ((hi - lo) * length > 1e-6)
  {
    if (f1 <= f2)
    {
      hi = x2;
      x2 = x1;
      f2 = f1;
      x1 = hi - ratio * (hi - lo);
      f1 = pointToBox(a + x1 * d, box).distance;
    }
    else
    {
      lo = x1;
      x1 = x2;
      f1 = f2;
      x2 = lo + ratio * (hi - lo);
      f2 = pointToBox(a + x2 * d, box).distance;
    }
  }

  result.t = 0.5 * (lo + hi);
  const PointBoxResult closest = pointToBox(a + result.t * d, box);
  result.distance = closest.distance;
  result.normal = closest.normal;
  return result;
}

SegmentSegmentResult segmentToSegment(const Eigen::Vector3d& a1, const Eigen::Vector3d& b1, const Eigen::Vector3d& a2,
                                      const Eigen::Vector3d& b2)
{
  const Eigen::Vector3d d1 = b1 - a1, d2 = b2 - a2, r = a1 - a2;
  const double a = d1.squaredNorm(), e = d2.squaredNorm(), f = d2.dot(r);
  const auto clamp01 = [](double x) { return std::min(1.0, std::max(0.0, x)); };
  constexpr double tiny = 1e-12;

  double s = 0.0, t = 0.0;
  if (a > tiny || e > tiny)
  {
    if (a <= tiny)
    {
      t = clamp01(f / e);
    }
    else
    {
      const double c = d1.dot(r);
      if (e <= tiny)
      {
        s = clamp01(-c / a);
      }
      else
      {
        const double b = d1.dot(d2);
        const double denominator = a * e - b * b;
        s = denominator > tiny ? clamp01((b * f - c * e) / denominator) : 0.0;
        t = (b * s + f) / e;
        if (t < 0.0)
        {
          t = 0.0;
          s = clamp01(-c / a);
        }
        else if (t > 1.0)
        {
          t = 1.0;
          s = clamp01((b - c) / a);
        }
      }
    }
  }

  SegmentSegmentResult result;
  result.s = s;
  result.t = t;
  const Eigen::Vector3d between = (a1 + s * d1) - (a2 + t * d2);
  result.distance = between.norm();
  if (result.distance > tiny)
  {
    result.normal = between / result.distance;
  }
  else
  {
    const Eigen::Vector3d cross = d1.cross(d2);
    result.normal = cross.norm() > tiny ? Eigen::Vector3d(cross.normalized()) : Eigen::Vector3d(Eigen::Vector3d::UnitX());
  }
  return result;
}

}  // namespace rmrrt

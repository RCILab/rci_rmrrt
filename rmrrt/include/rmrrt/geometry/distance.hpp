#pragma once

#include <Eigen/Geometry>

namespace rmrrt {

// An oriented box: pose of its center and half sizes.
struct Box
{
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d half{Eigen::Vector3d::Zero()};
};

struct PointBoxResult
{
  double distance{0.0};    // signed: negative inside
  Eigen::Vector3d normal;  // gradient of the distance with respect to the point
};

struct SegmentBoxResult
{
  double distance{0.0};    // signed
  double t{0.0};           // closest point = a + t (b - a)
  Eigen::Vector3d normal;  // at the closest point
  bool exact{true};        // false: only a lower bound >= cutoff, t and normal are not set
};

struct SegmentSegmentResult
{
  double distance{0.0};
  double s{0.0};           // closest point on the first segment  = a1 + s (b1 - a1)
  double t{0.0};           // closest point on the second segment = a2 + t (b2 - a2)
  Eigen::Vector3d normal;  // from the second closest point toward the first
};

PointBoxResult pointToBox(const Eigen::Vector3d& point, const Box& box);

// Golden-section search along the segment: the signed distance to a convex set is convex along it.
// Skipped when a lower bound already reaches cutoff.
SegmentBoxResult segmentToBox(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Box& box, double cutoff);

SegmentSegmentResult segmentToSegment(const Eigen::Vector3d& a1, const Eigen::Vector3d& b1, const Eigen::Vector3d& a2,
                                      const Eigen::Vector3d& b2);

}  // namespace rmrrt

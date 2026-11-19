#include <random>

#include <gtest/gtest.h>

#include "rmrrt/geometry/capsule_margins.hpp"

using namespace rmrrt;

namespace {

// One link translated by q (n = 3), not rotated.
class TranslatingLink : public KinematicsBase
{
public:
  int dim() const override { return 3; }
  void setConfiguration(ConstRefVector q) const override { position_ = q; }
  Eigen::Isometry3d linkPose(int) const override
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = position_;
    return pose;
  }
  Eigen::Matrix<double, 3, Eigen::Dynamic> pointJacobian(int, const Eigen::Vector3d&) const override { return Eigen::Matrix3d::Identity(); }

private:
  mutable Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
};

double bruteSegmentSegment(const Eigen::Vector3d& a1, const Eigen::Vector3d& b1, const Eigen::Vector3d& a2, const Eigen::Vector3d& b2)
{
  double best = 1e9;
  for (int i = 0; i <= 400; ++i)
    for (int j = 0; j <= 400; ++j)
      best = std::min(best, ((a1 + (b1 - a1) * (i / 400.0)) - (a2 + (b2 - a2) * (j / 400.0))).norm());
  return best;
}

}  // namespace

TEST(Distance, SegmentToSegmentMatchesSampling)
{
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const auto point = [&] { return Eigen::Vector3d(u(rng), u(rng), u(rng)); };
  for (int k = 0; k < 20; ++k)
  {
    const Eigen::Vector3d a1 = point(), b1 = point(), a2 = point(), b2 = point();
    EXPECT_NEAR(segmentToSegment(a1, b1, a2, b2).distance, bruteSegmentSegment(a1, b1, a2, b2), 5e-3);
  }
}

TEST(Distance, PointToBoxIsSigned)
{
  Box box;
  box.half = Eigen::Vector3d(0.5, 0.2, 0.1);
  EXPECT_NEAR(pointToBox(Eigen::Vector3d(1.0, 0.0, 0.0), box).distance, 0.5, 1e-12);
  EXPECT_NEAR(pointToBox(Eigen::Vector3d(0.0, 0.0, 0.05), box).distance, -0.05, 1e-12);
  EXPECT_NEAR(pointToBox(Eigen::Vector3d(0.8, 0.6, 0.0), box).distance, 0.5, 1e-12);
}

TEST(Distance, SegmentToBoxMatchesSampling)
{
  Box box;
  box.pose.translation() = Eigen::Vector3d(0.2, -0.1, 0.3);
  box.pose.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  box.half = Eigen::Vector3d(0.3, 0.1, 0.2);
  const Eigen::Vector3d a(-1.0, 0.4, 0.2), b(1.0, -0.3, 0.9);
  double best = 1e9;
  for (int i = 0; i <= 20000; ++i)
    best = std::min(best, pointToBox(a + (b - a) * (i / 20000.0), box).distance);
  EXPECT_NEAR(segmentToBox(a, b, box, 1e9).distance, best, 1e-5);
}

TEST(CapsuleMargins, ObstacleGradientMatchesFiniteDifferences)
{
  const TranslatingLink link;
  Capsule capsule;
  capsule.a = Eigen::Vector3d(-0.1, 0.0, 0.0);
  capsule.b = Eigen::Vector3d(0.1, 0.05, 0.0);
  capsule.radius = 0.03;
  Box box;
  box.pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.2);
  box.half = Eigen::Vector3d(0.2, 0.2, 0.05);
  const CapsuleMargins margins(link, {capsule}, {}, {box}, Vector::Constant(3, -10.0), Vector::Constant(3, 10.0));
  const Barrier barrier;
  const Vector q = Eigen::Vector3d(0.01, -0.02, 0.03);
  Margins at_q;
  margins.compute(q, barrier, at_q);
  ASSERT_EQ(at_q.size(), 1u);
  for (int i = 0; i < 3; ++i)
  {
    Margins plus, minus;
    margins.compute(q + 1e-6 * Vector::Unit(3, i), barrier, plus);
    margins.compute(q - 1e-6 * Vector::Unit(3, i), barrier, minus);
    EXPECT_NEAR((plus[0].h - minus[0].h) / 2e-6, at_q[0].gradient[i], 1e-4);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>

#include "rmrrt/planner/newton_projection.hpp"
#include "rmrrt/steering/steering_factory.hpp"
#include "test_problems.hpp"

using namespace rmrrt;
using namespace rmrrt::test;

namespace {

Vector target()
{
  Vector t(6);
  t << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;
  return t;
}

}  // namespace

TEST(Steering, EveryStepHasLengthEtaAndStaysTangent)
{
  Vector gradient(6);
  gradient << 0.0, 0.6, 0.8, 0.0, 0.0, 0.0;
  const OrthonormalPair problem({obstacleMargin(0.04, gradient)});
  const RiemannianMetric metric;
  const Vector q = orthonormalStart();
  for (const char* type : {"euclidean", "metric", "centered"})
  {
    const auto steering = createSteering(type);
    steering->setStepSize(0.1);
    Chart chart;
    ASSERT_TRUE(metric.computeChart(problem, q, steering->usesBarrier(), chart));
    Vector next;
    ASSERT_TRUE(steering->step(chart, q, target(), next)) << type;
    EXPECT_NEAR((next - q).norm(), 0.1, 1e-12) << type;
    EXPECT_LT((problem.J(q) * (next - q)).norm(), 1e-10) << type;
  }
}

TEST(Steering, CenteredStepMovesAwayFromTheBound)
{
  Vector gradient(6);
  gradient << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;
  const OrthonormalPair problem({obstacleMargin(0.02, gradient)});
  const RiemannianMetric metric;
  const Vector q = orthonormalStart();
  Chart chart;
  ASSERT_TRUE(metric.computeChart(problem, q, true, chart));
  const auto metric_steering = createSteering("metric");
  const auto centered = createSteering("centered", 0.5);
  Vector plain, pushed;
  ASSERT_TRUE(metric_steering->step(chart, q, target(), plain));
  ASSERT_TRUE(centered->step(chart, q, target(), pushed));
  EXPECT_GT(gradient.dot(pushed - q), gradient.dot(plain - q));
}

TEST(Steering, NoDirectionTowardTheSamePoint)
{
  const OrthonormalPair problem;
  const RiemannianMetric metric;
  const Vector q = orthonormalStart();
  Chart chart;
  ASSERT_TRUE(metric.computeChart(problem, q, true, chart));
  Vector next;
  EXPECT_FALSE(createSteering("centered")->step(chart, q, q, next));
}

TEST(NewtonProjection, ReachesTheManifold)
{
  const OrthonormalPair problem;
  const RiemannianMetric metric;
  NewtonProjection projection(problem, metric);
  Vector q = orthonormalStart();
  q += 0.05 * Vector::Ones(6);
  ASSERT_TRUE(projection.project(q, false));
  EXPECT_LT(problem.g(q).norm(), projection.tolerance());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

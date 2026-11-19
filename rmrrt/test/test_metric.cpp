#include <gtest/gtest.h>

#include "rmrrt/metric/riemannian_metric.hpp"
#include "test_problems.hpp"

using namespace rmrrt;
using namespace rmrrt::test;

TEST(Barrier, WeightAndPushVanishAtTheWidth)
{
  const Barrier barrier;
  const Vector gradient = Vector::Unit(6, 0);
  EXPECT_EQ(barrier.weight(obstacleMargin(0.15, gradient)), 0.0);
  EXPECT_NEAR(barrier.push(obstacleMargin(0.15 - 1e-9, gradient)), 0.0, 1e-6);
  EXPECT_GT(barrier.weight(obstacleMargin(0.05, gradient)), barrier.weight(obstacleMargin(0.10, gradient)));
  EXPECT_GT(barrier.push(obstacleMargin(0.05, gradient)), barrier.push(obstacleMargin(0.10, gradient)));
}

TEST(Barrier, WidthZeroRemovesTheCategory)
{
  Barrier barrier;
  barrier.setWidth(Category::Obstacle, 0.0);
  EXPECT_FALSE(barrier.enabled(Category::Obstacle));
  EXPECT_EQ(barrier.weight(obstacleMargin(0.01, Vector::Unit(6, 0))), 0.0);
}

TEST(RiemannianMetric, ProjectorIsTangentAndGOrthogonal)
{
  Vector gradient(6);
  gradient << 0.3, -0.2, 0.5, 0.1, 0.4, -0.6;
  const OrthonormalPair problem({obstacleMargin(0.02, gradient)});
  const RiemannianMetric metric;
  const Vector q = orthonormalStart();
  Chart chart;
  ASSERT_TRUE(metric.computeChart(problem, q, true, chart));
  const Matrix J = problem.J(q);
  EXPECT_LT((J * chart.P).norm(), 1e-10);                                      // onto the tangent space
  EXPECT_LT((chart.P * chart.P - chart.P).norm(), 1e-10);                      // a projector
  EXPECT_LT((chart.G * chart.P - chart.P.transpose() * chart.G).norm(), 1e-8);  // G-orthogonal
}

TEST(RiemannianMetric, QInvertsMOnTheTangentSpace)
{
  Vector gradient(6);
  gradient << -0.1, 0.7, 0.2, 0.3, -0.5, 0.4;
  const OrthonormalPair problem({obstacleMargin(0.03, gradient)});
  const RiemannianMetric metric;
  const Vector q = orthonormalStart();
  Chart chart;
  ASSERT_TRUE(metric.computeChart(problem, q, true, chart));
  const Matrix M = chart.P.transpose() * chart.G * chart.P;
  EXPECT_LT((M * chart.Q - chart.P.transpose()).norm(), 1e-8);
  EXPECT_LT((problem.J(q) * chart.Q).norm(), 1e-10);
}

TEST(RiemannianMetric, WithoutBarrierGIsLambdaIdentity)
{
  const OrthonormalPair problem({obstacleMargin(0.01, Vector::Unit(6, 2))});
  RiemannianMetric metric;
  metric.setLambda(2.0);
  Chart chart;
  ASSERT_TRUE(metric.computeChart(problem, orthonormalStart(), false, chart));
  EXPECT_LT((chart.G - 2.0 * Matrix::Identity(6, 6)).norm(), 1e-12);
  EXPECT_EQ(chart.force.norm(), 0.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

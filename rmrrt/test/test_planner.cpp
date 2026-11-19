#include <gtest/gtest.h>

#include "../examples/sphere/sphere_problem.hpp"

using namespace rmrrt;
using namespace rmrrt::sphere;

TEST(RRTConnect, EveryMethodSolvesAnOpenSphere)
{
  const SphereInstance instance = makeInstance("open", 0.0, 0);
  const SphereProblem problem(instance.centers, instance.radii);
  const RiemannianMetric metric;
  for (const Method& method : methods())
  {
    RRTConnect planner(problem, metric);
    setUp(planner, method, 0);
    EXPECT_TRUE(planner.solve(instance.start, instance.goal, 60.0)) << method.name;
  }
}

TEST(RRTConnect, PathIsValidAndOnTheManifold)
{
  const SphereInstance instance = makeInstance("double_gate", 0.15, 5);
  const SphereProblem problem(instance.centers, instance.radii);
  const RiemannianMetric metric;
  RRTConnect planner(problem, metric);
  setUp(planner, findMethod("rmrrt"), 5);
  ASSERT_TRUE(planner.solve(instance.start, instance.goal, 60.0));
  const std::vector<Vector>& path = planner.path();
  EXPECT_LT((path.front() - instance.start).norm(), 1e-2);
  EXPECT_LT((path.back() - instance.goal).norm(), 1e-2);
  for (const Vector& q : path)
  {
    EXPECT_TRUE(problem.isValid(q));
    EXPECT_LT(problem.g(q).norm(), 1e-3);
  }
}

TEST(RRTConnect, SameSeedSameSearch)
{
  const SphereInstance instance = makeInstance("gate", 0.25, 2);
  const SphereProblem problem(instance.centers, instance.radii);
  const RiemannianMetric metric;
  std::size_t checks[2];
  for (std::size_t& c : checks)
  {
    RRTConnect planner(problem, metric);
    setUp(planner, findMethod("rmrrt"), 2);
    planner.solve(instance.start, instance.goal, 60.0);
    c = planner.statistics().checks;
  }
  EXPECT_EQ(checks[0], checks[1]);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

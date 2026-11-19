#pragma once

#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>

#include "rmrrt/robot/scenario.hpp"

namespace rmrrt {
namespace robot {

// A scenario of config/scenarios.yaml and its queries, scenes/<scenario>/scene_NNNN/ with scene.yaml (the
// boxes), start_q.txt and goal_q.txt. Paths are relative to the rmrrt package.
class Experiment
{
public:
  explicit Experiment(const std::string& scenario);

  const ScenarioSpec& spec() const { return spec_; }
  // The scene numbers, in order.
  const std::vector<int>& scenes() const { return scenes_; }
  std::string sceneFile(int scene) const;
  Vector start(int scene) const;
  Vector goal(int scene) const;
  // The random seed of a query: the same for every method.
  static unsigned seed(unsigned base, int scene) { return base * 100000u + static_cast<unsigned>(scene); }

  // The robot on the ROS parameter server; says which launch file loads it when it is missing.
  moveit::core::RobotModelConstPtr loadRobot() const;
  // results/<scenario>/<file>, the folder created.
  std::string resultPath(const std::string& file) const;
  static std::string packagePath(const std::string& relative);

private:
  std::string sceneDir(int scene) const;

  ScenarioSpec spec_;
  std::string folder_;
  std::vector<int> scenes_;
};

Vector loadVector(const std::string& file);

}  // namespace robot
}  // namespace rmrrt

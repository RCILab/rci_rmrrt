#include "rmrrt/robot/experiment.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/master.h>
#include <ros/package.h>
#include <ros/param.h>

namespace rmrrt {
namespace robot {

namespace fs = std::filesystem;

Vector loadVector(const std::string& file)
{
  std::ifstream in(file);
  if (!in)
    throw std::runtime_error("Cannot read " + file);
  const std::vector<double> values{std::istream_iterator<double>(in), std::istream_iterator<double>()};
  return Eigen::Map<const Vector>(values.data(), static_cast<int>(values.size()));
}

Experiment::Experiment(const std::string& scenario)
  : spec_(loadScenario(packagePath("config/scenarios.yaml"), scenario)), folder_(packagePath("scenes/" + scenario))
{
  if (!fs::is_directory(folder_))
    throw std::runtime_error("No scene folder " + folder_);
  for (const auto& entry : fs::directory_iterator(folder_))
  {
    const std::string name = entry.path().filename().string();
    if (entry.is_directory() && name.rfind("scene_", 0) == 0)
      scenes_.push_back(std::stoi(name.substr(6)));
  }
  std::sort(scenes_.begin(), scenes_.end());
}

std::string Experiment::sceneFile(int scene) const
{
  return sceneDir(scene) + "/scene.yaml";
}

Vector Experiment::start(int scene) const
{
  return loadVector(sceneDir(scene) + "/start_q.txt");
}

Vector Experiment::goal(int scene) const
{
  return loadVector(sceneDir(scene) + "/goal_q.txt");
}

moveit::core::RobotModelConstPtr Experiment::loadRobot() const
{
  const std::string hint = "Load the robot of " + spec_.name + " first:\n  roslaunch rmrrt robot.launch scenario:=" + spec_.name;
  if (!ros::master::check() || !ros::param::has("/robot_description_semantic"))
    throw std::runtime_error("No robot on the ROS parameter server. " + hint);
  static robot_model_loader::RobotModelLoader loader("robot_description", false);
  const auto model = loader.getModel();
  if (!model)
    throw std::runtime_error("Cannot load /robot_description. " + hint);
  for (const std::string& group : spec_.groups)
    if (!model->hasJointModelGroup(group))
      throw std::runtime_error("The loaded robot " + model->getName() + " has no group " + group + ". " + hint);
  return model;
}

std::string Experiment::resultPath(const std::string& file) const
{
  const fs::path folder = packagePath("results/" + spec_.name);
  fs::create_directories(folder);
  return (folder / file).string();
}

std::string Experiment::packagePath(const std::string& relative)
{
  const std::string root = ros::package::getPath("rmrrt");
  if (root.empty())
    throw std::runtime_error("Package rmrrt not found: source the workspace (devel/setup.bash)");
  return (fs::path(root) / relative).lexically_normal().string();
}

std::string Experiment::sceneDir(int scene) const
{
  std::ostringstream dir;
  dir << folder_ << "/scene_" << std::setw(4) << std::setfill('0') << scene;
  return dir.str();
}

}  // namespace robot
}  // namespace rmrrt

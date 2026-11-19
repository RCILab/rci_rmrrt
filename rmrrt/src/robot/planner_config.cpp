#include "rmrrt/robot/planner_config.hpp"

#include <yaml-cpp/yaml.h>

namespace rmrrt {
namespace robot {

PlannerConfig PlannerConfig::load(const std::string& file)
{
  const YAML::Node root = YAML::LoadFile(file);
  const YAML::Node planner = root["planner"], metric = root["metric"], width = root["metric"]["barrier_width"];
  PlannerConfig config;
  config.step_size = planner["step_size"].as<double>();
  config.connection_tolerance = planner["connection_tolerance"].as<double>();
  config.projection_tolerance = planner["projection_tolerance"].as<double>();
  config.edge_resolution = planner["edge_resolution"].as<double>();
  config.timeout = planner["timeout"].as<double>();
  config.seed = planner["seed"].as<unsigned>();
  config.lambda = metric["lambda"].as<double>();
  config.beta = metric["beta"].as<double>();
  config.obstacle_width = width["obstacle"].as<double>();
  config.self_collision_width = width["self_collision"].as<double>();
  config.joint_limit_width = width["joint_limit"].as<double>();
  config.epsilon = metric["epsilon"].as<double>();
  config.psi_max = metric["psi_max"].as<double>();
  return config;
}

void PlannerConfig::apply(RiemannianMetric& metric) const
{
  metric.setLambda(lambda);
  metric.barrier().setWidth(Category::Obstacle, obstacle_width);
  metric.barrier().setWidth(Category::SelfCollision, self_collision_width);
  metric.barrier().setWidth(Category::JointLimit, joint_limit_width);
  metric.barrier().setEpsilon(epsilon);
  metric.barrier().setPsiMax(psi_max);
}

void PlannerConfig::apply(RRTConnect& planner, const Method& method) const
{
  configure(planner, method, beta);
  planner.setStepSize(step_size);
  planner.setConnectionTolerance(connection_tolerance);
  planner.setEdgeResolution(edge_resolution);
  planner.projection().setTolerance(projection_tolerance);
}

}  // namespace robot
}  // namespace rmrrt

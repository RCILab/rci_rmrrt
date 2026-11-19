#include "rmrrt/robot/scenario.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace rmrrt {
namespace robot {

namespace {

Eigen::Vector3d vector3(const YAML::Node& node)
{
  const auto v = node.as<std::vector<double>>();
  if (v.size() != 3)
    throw std::runtime_error("Expected a 3-vector in the scenario file");
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

// [ax, ay, az, bx, by, bz, radius]
Capsule capsule(const YAML::Node& row)
{
  const auto v = row.as<std::vector<double>>();
  Capsule c;
  c.a = Eigen::Vector3d(v[0], v[1], v[2]);
  c.b = Eigen::Vector3d(v[3], v[4], v[5]);
  c.radius = v[6];
  return c;
}

TableSpec table(const YAML::Node& item)
{
  TableSpec t;
  t.id = item["id"].as<std::string>();
  t.center = vector3(item["center"]);
  t.length = item["length"].as<double>();
  t.width = item["width"].as<double>();
  t.height = item["height"].as<double>();
  t.thickness = item["thickness"].as<double>();
  return t;
}

ShelfSpec shelf(const YAML::Node& item)
{
  ShelfSpec s;
  s.id = item["id"].as<std::string>();
  s.center = vector3(item["center"]);
  s.yaw = item["yaw"].as<double>();
  s.length = item["length"].as<double>();
  s.width = item["width"].as<double>();
  s.height = item["height"].as<double>();
  s.thickness = item["thickness"].as<double>();
  s.parts = item["parts"].as<int>();
  return s;
}

BoxSpec box(const YAML::Node& item)
{
  BoxSpec b;
  b.id = item["id"].as<std::string>();
  b.center = vector3(item["center"]);
  b.size = vector3(item["size"]);
  return b;
}

CarriedSpec carried(const YAML::Node& item)
{
  CarriedSpec c;
  c.id = item["id"].as<std::string>();
  c.link = item["link"].as<std::string>();
  const std::string shape = item["shape"].as<std::string>();
  if (shape == "box")
  {
    c.shape = CarriedSpec::Shape::Box;
    c.size = vector3(item["size"]);
  }
  else if (shape == "cylinder")
  {
    c.shape = CarriedSpec::Shape::Cylinder;
    c.radius = item["radius"].as<double>();
    c.length = item["length"].as<double>();
  }
  else if (shape == "mesh")
  {
    c.shape = CarriedSpec::Shape::Mesh;
    c.mesh = item["mesh"].as<std::string>();
    for (const auto& row : item["capsules"])
      c.mesh_capsules.push_back(capsule(row));
  }
  else
  {
    throw std::runtime_error("Unknown carried shape " + shape + " (box | cylinder | mesh)");
  }
  const auto q = item["quat_wxyz"].as<std::vector<double>>();
  c.pose.translation() = vector3(item["position"]);
  c.pose.linear() = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
  if (item["touch_links"])
    c.touch_links = item["touch_links"].as<std::vector<std::string>>();
  if (item["ignore_links"])
    c.ignore_links = item["ignore_links"].as<std::vector<std::string>>();
  return c;
}

}  // namespace

ScenarioSpec loadScenario(const std::string& file, const std::string& name)
{
  const YAML::Node root = YAML::LoadFile(file);
  const YAML::Node node = root["scenarios"][name];
  if (!node)
  {
    std::string known;
    for (const auto& entry : root["scenarios"])
      known += (known.empty() ? "" : " | ") + entry.first.as<std::string>();
    throw std::runtime_error("Unknown scenario " + name + " (" + known + ")");
  }

  ScenarioSpec spec;
  spec.name = name;
  spec.groups = node["groups"].as<std::vector<std::string>>();
  spec.tips = node["tips"].as<std::vector<std::string>>();
  const auto list = [&](const char* key) { return node[key] ? node[key] : YAML::Node(YAML::NodeType::Sequence); };
  for (const auto& item : list("tables"))
    spec.tables.push_back(table(item));
  for (const auto& item : list("shelves"))
    spec.shelves.push_back(shelf(item));
  for (const auto& item : list("boxes"))
    spec.boxes.push_back(box(item));
  if (node["world_box_links"])
    spec.world_box_links = node["world_box_links"].as<std::vector<std::string>>();
  if (node["link_capsules"])
    for (const auto& entry : node["link_capsules"])
      for (const auto& row : entry.second)
        spec.link_capsules[entry.first.as<std::string>()].push_back(capsule(row));
  if (node["fixed_joints"])
    spec.fixed_joints = node["fixed_joints"].as<std::map<std::string, double>>();

  if (node["carried"])
  {
    spec.has_carried = true;
    spec.carried = carried(node["carried"]);
  }
  if (const YAML::Node item = node["keep_direction"])
  {
    spec.has_direction = true;
    spec.direction.link = item["link"].as<std::string>();
    spec.direction.local = vector3(item["local"]).normalized();
    spec.direction.world = vector3(item["world"]).normalized();
    if (item["in_carried"] && item["in_carried"].as<bool>())  // the axis is given in the frame of the carried object
      spec.direction.local = spec.carried.pose.linear() * spec.direction.local;
  }
  return spec;
}

}  // namespace robot
}  // namespace rmrrt

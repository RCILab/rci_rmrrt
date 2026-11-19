#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "rmrrt/geometry/capsule_margins.hpp"

namespace rmrrt {
namespace robot {

// A table: a top plate on four legs, `center` at the middle of its height.
struct TableSpec
{
  std::string id;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double length{0.0}, width{0.0}, height{0.0}, thickness{0.0};
};

// A shelf: top, bottom, sides and back with `parts` compartments, turned by `yaw` about its center.
struct ShelfSpec
{
  std::string id;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double yaw{0.0}, length{0.0}, width{0.0}, height{0.0}, thickness{0.0};
  int parts{1};
};

struct BoxSpec
{
  std::string id;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
};

// The object held by the robot. The validity check uses its shape, the metric its capsules.
struct CarriedSpec
{
  enum class Shape
  {
    Box,
    Cylinder,
    Mesh
  };

  std::string id;
  std::string link;  // the link it is attached to
  Shape shape{Shape::Box};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};          // box
  double radius{0.0}, length{0.0};                        // cylinder along its z axis
  std::string mesh;                                       // mesh resource
  std::vector<Capsule> mesh_capsules;                     // capsules of the mesh, in the mesh frame
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};  // in the frame of the link
  std::vector<std::string> touch_links;                   // contacts allowed by the grasp
  std::vector<std::string> ignore_links;                  // capsules it touches by construction
};

// A tool axis kept along a world axis: two equality constraints.
struct DirectionSpec
{
  std::string link;
  Eigen::Vector3d local{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d world{Eigen::Vector3d::UnitZ()};
};

// One scenario of config/scenarios.yaml.
struct ScenarioSpec
{
  std::string name;
  std::vector<std::string> groups;  // the order of the joints in q
  std::vector<std::string> tips;    // tool frames held together by the carried object
  std::vector<TableSpec> tables;
  std::vector<ShelfSpec> shelves;
  std::vector<BoxSpec> boxes;
  std::vector<std::string> world_box_links;  // robot links whose boxes the metric treats as obstacles
  std::map<std::string, std::vector<Capsule>> link_capsules;  // capsules of mesh links, in the link frame
  std::map<std::string, double> fixed_joints;
  bool has_carried{false};
  CarriedSpec carried;
  bool has_direction{false};
  DirectionSpec direction;
};

ScenarioSpec loadScenario(const std::string& file, const std::string& name);

}  // namespace robot
}  // namespace rmrrt

#include "rmrrt/robot/robot_scene.hpp"

#include <stdexcept>

#include <boost/variant/get.hpp>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit/robot_state/conversions.h>
#include <urdf_model/model.h>
#include <yaml-cpp/yaml.h>

namespace rmrrt {
namespace robot {

namespace {

Box axisAlignedBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size)
{
  Box box;
  box.pose.translation() = center;
  box.half = 0.5 * size;
  return box;
}

std::vector<std::pair<std::string, Box>> tableBoxes(const TableSpec& table)
{
  const double lx = 0.5 * (table.length - table.thickness), ly = 0.5 * (table.width - table.thickness);
  const Eigen::Vector3d leg(table.thickness, table.thickness, table.height);
  const Eigen::Vector3d c = table.center;
  return {
      {table.id + "_top", axisAlignedBox(c + Eigen::Vector3d(0, 0, 0.5 * (table.height - table.thickness)),
                                         Eigen::Vector3d(table.length, table.width, table.thickness))},
      {table.id + "_leg0", axisAlignedBox(c + Eigen::Vector3d(-lx, ly, 0), leg)},
      {table.id + "_leg1", axisAlignedBox(c + Eigen::Vector3d(-lx, -ly, 0), leg)},
      {table.id + "_leg2", axisAlignedBox(c + Eigen::Vector3d(lx, ly, 0), leg)},
      {table.id + "_leg3", axisAlignedBox(c + Eigen::Vector3d(lx, -ly, 0), leg)},
  };
}

std::vector<std::pair<std::string, Box>> shelfBoxes(const ShelfSpec& shelf)
{
  const Eigen::AngleAxisd turn(shelf.yaw, Eigen::Vector3d::UnitZ());
  const auto part = [&](const Eigen::Vector3d& offset, const Eigen::Vector3d& size) {
    Box box;
    box.pose.translation() = shelf.center + turn * offset;
    box.pose.linear() = turn.toRotationMatrix();
    box.half = 0.5 * size;
    return box;
  };
  const double l = shelf.length, w = shelf.width, h = shelf.height, d = shelf.thickness;
  const Eigen::Vector3d plate(l, w, d), side(d, w, h), back(l, d, h);
  std::vector<std::pair<std::string, Box>> boxes{
      {shelf.id + "_top", part(Eigen::Vector3d(0, 0, 0.5 * (h - d)), plate)},
      {shelf.id + "_bottom", part(Eigen::Vector3d(0, 0, -0.5 * (h - d)), plate)},
      {shelf.id + "_left", part(Eigen::Vector3d(-0.5 * (l - d), 0, 0), side)},
      {shelf.id + "_right", part(Eigen::Vector3d(0.5 * (l - d), 0, 0), side)},
      {shelf.id + "_back", part(Eigen::Vector3d(0, 0.5 * (w - d), 0), back)},
  };
  for (int i = 1; i < shelf.parts; ++i)
    boxes.emplace_back(shelf.id + "_mid" + std::to_string(i), part(Eigen::Vector3d(0, 0, -0.5 * (h - d) + i * (h - d) / shelf.parts), plate));
  return boxes;
}

geometry_msgs::Pose poseMessage(const Eigen::Isometry3d& pose)
{
  geometry_msgs::Pose out;
  out.position.x = pose.translation().x();
  out.position.y = pose.translation().y();
  out.position.z = pose.translation().z();
  const Eigen::Quaterniond orientation(pose.linear());
  out.orientation.w = orientation.w();
  out.orientation.x = orientation.x();
  out.orientation.y = orientation.y();
  out.orientation.z = orientation.z();
  return out;
}

Eigen::Isometry3d urdfPose(const urdf::Pose& origin)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(origin.position.x, origin.position.y, origin.position.z);
  pose.linear() = Eigen::Quaterniond(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z).toRotationMatrix();
  return pose;
}

}  // namespace

RigidBodies::RigidBodies(const moveit::core::RobotModelConstPtr& model, const std::vector<std::string>& groups) : model_(model)
{
  for (const std::string& name : groups)
    for (const auto* joint : model->getJointModelGroup(name)->getActiveJointModels())
      planned_.insert(joint);
}

std::string RigidBodies::root(const std::string& name) const
{
  const moveit::core::LinkModel* link = model_->getLinkModel(name);
  while (link->getParentLinkModel() && !planned_.count(link->getParentJointModel()))
    link = link->getParentLinkModel();
  return link->getName();
}

RobotScene::RobotScene(const moveit::core::RobotModelConstPtr& model, const ScenarioSpec& spec, const std::string& scene_file)
  : model_(model), scene_(std::make_shared<planning_scene::PlanningScene>(model))
{
  auto& state = scene_->getCurrentStateNonConst();
  state.setToDefaultValues();
  for (const auto& [joint, value] : spec.fixed_joints)
    if (model->hasJointModel(joint))
      state.setJointPositions(joint, &value);

  for (const TableSpec& table : spec.tables)
    for (const auto& [id, box] : tableBoxes(table))
      addBox(id, box);
  addSceneFile(scene_file);
  for (const ShelfSpec& shelf : spec.shelves)
    for (const auto& [id, box] : shelfBoxes(shelf))
      addBox(id, box);
  for (const BoxSpec& box : spec.boxes)
    addBox(box.id, axisAlignedBox(box.center, box.size));
  state.update();
  addWorldBoxLinks(spec.world_box_links);

  if (spec.has_carried)
    attach(spec.carried);
  state.update();
  allowRigidContacts(spec);
}

bool RobotScene::isValid(const moveit::core::RobotState& state) const
{
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  scene_->checkCollision(request, result, state);
  return !result.collision;
}

std::string RobotScene::describeContact(const moveit::core::RobotState& state) const
{
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  request.contacts = true;
  request.max_contacts = 1;
  request.max_contacts_per_pair = 1;
  scene_->checkCollision(request, result, state);
  if (!result.collision || result.contacts.empty())
    return "";
  return "contact " + result.contacts.begin()->first.first + " - " + result.contacts.begin()->first.second;
}

void RobotScene::message(const moveit::core::RobotState& state, moveit_msgs::PlanningScene& out) const
{
  scene_->getPlanningSceneMsg(out);
  moveit::core::robotStateToRobotStateMsg(state, out.robot_state);
  out.is_diff = false;
}

void RobotScene::addBox(const std::string& id, const Box& box)
{
  moveit_msgs::CollisionObject object;
  object.id = id;
  object.header.frame_id = scene_->getPlanningFrame();
  object.operation = moveit_msgs::CollisionObject::ADD;
  object.pose.orientation.w = 1.0;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions = {2.0 * box.half.x(), 2.0 * box.half.y(), 2.0 * box.half.z()};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(poseMessage(box.pose));
  if (!scene_->processCollisionObjectMsg(object))
    throw std::runtime_error("Failed to add " + id + " to the scene");
  obstacles_.push_back(box);
}

// {name: {dim: [x, y, z], pos: [x, y, z]}, ...}: axis-aligned boxes of this query.
void RobotScene::addSceneFile(const std::string& file)
{
  if (file.empty())
    return;
  const YAML::Node boxes = YAML::LoadFile(file);
  for (auto it = boxes.begin(); it != boxes.end(); ++it)
  {
    const YAML::Node item = it->second;
    if (!item.IsMap() || !item["dim"] || !item["pos"] || item["dim"].size() != 3)
      continue;
    const auto size = item["dim"].as<std::vector<double>>(), center = item["pos"].as<std::vector<double>>();
    addBox(it->first.as<std::string>(), axisAlignedBox(Eigen::Vector3d(center[0], center[1], center[2]), Eigen::Vector3d(size[0], size[1], size[2])));
  }
}

// Boxes of fixed robot links (a mounting plate, say). The robot model already holds them; the metric
// treats them as obstacles.
void RobotScene::addWorldBoxLinks(const std::vector<std::string>& links)
{
  const auto& state = scene_->getCurrentState();
  for (const std::string& name : links)
  {
    const auto link = model_->getURDF()->getLink(name);
    if (!link)
      throw std::runtime_error("Unknown link " + name);
    for (const auto& collision : link->collision_array)
      if (collision->geometry && collision->geometry->type == urdf::Geometry::BOX)
      {
        const auto* shape = static_cast<const urdf::Box*>(collision->geometry.get());
        Box box;
        box.pose = state.getGlobalLinkTransform(name) * urdfPose(collision->origin);
        box.half = 0.5 * Eigen::Vector3d(shape->dim.x, shape->dim.y, shape->dim.z);
        obstacles_.push_back(box);
      }
  }
}

void RobotScene::attach(const CarriedSpec& carried)
{
  moveit_msgs::AttachedCollisionObject attached;
  attached.link_name = carried.link;
  attached.touch_links = carried.touch_links;
  attached.object.id = carried.id;
  attached.object.header.frame_id = carried.link;
  attached.object.operation = moveit_msgs::CollisionObject::ADD;
  attached.object.pose.orientation.w = 1.0;
  if (carried.shape == CarriedSpec::Shape::Mesh)
  {
    const std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(carried.mesh));
    if (!mesh)
      throw std::runtime_error("Cannot load " + carried.mesh);
    shapes::ShapeMsg message;
    shapes::constructMsgFromShape(mesh.get(), message);
    attached.object.meshes.push_back(boost::get<shape_msgs::Mesh>(message));
    attached.object.mesh_poses.push_back(poseMessage(carried.pose));
  }
  else
  {
    shape_msgs::SolidPrimitive primitive;
    if (carried.shape == CarriedSpec::Shape::Box)
    {
      primitive.type = shape_msgs::SolidPrimitive::BOX;
      primitive.dimensions = {carried.size.x(), carried.size.y(), carried.size.z()};
    }
    else
    {
      primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
      primitive.dimensions = {carried.length, carried.radius};
    }
    attached.object.primitives.push_back(primitive);
    attached.object.primitive_poses.push_back(poseMessage(carried.pose));
  }
  if (!scene_->processAttachedCollisionObjectMsg(attached))
    throw std::runtime_error("Failed to attach " + carried.id);
}

// Contacts within one rigid body, and between the carried object and the body that holds it, are
// permanent: the validity check ignores them.
void RobotScene::allowRigidContacts(const ScenarioSpec& spec)
{
  const RigidBodies bodies(model_, spec.groups);
  auto& allowed = scene_->getAllowedCollisionMatrixNonConst();
  const auto& links = model_->getLinkModelsWithCollisionGeometry();
  for (std::size_t i = 0; i < links.size(); ++i)
  {
    for (std::size_t j = i + 1; j < links.size(); ++j)
      if (bodies.root(links[i]->getName()) == bodies.root(links[j]->getName()))
        allowed.setEntry(links[i]->getName(), links[j]->getName(), true);
    if (spec.has_carried && bodies.root(links[i]->getName()) == bodies.root(spec.carried.link))
      allowed.setEntry(spec.carried.id, links[i]->getName(), true);
  }
}

}  // namespace robot
}  // namespace rmrrt

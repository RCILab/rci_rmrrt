#pragma once

#include <set>
#include <string>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>

#include "rmrrt/robot/scenario.hpp"

namespace rmrrt {
namespace robot {

// Which rigid body a link belongs to: links joined only through joints that are not planned never move
// relative to each other, so a contact between them is permanent (a hand and its fixed fingers).
class RigidBodies
{
public:
  RigidBodies(const moveit::core::RobotModelConstPtr& model, const std::vector<std::string>& groups);

  // The link at the root of the body.
  std::string root(const std::string& link) const;

private:
  moveit::core::RobotModelConstPtr model_;
  std::set<const moveit::core::JointModel*> planned_;
};

// The MoveIt planning scene of one query: the scenario's obstacles, the boxes of the scene file and the
// carried object. It decides validity; the metric sees the same obstacles as boxes.
class RobotScene
{
public:
  RobotScene(const moveit::core::RobotModelConstPtr& model, const ScenarioSpec& spec, const std::string& scene_file);

  // No collision. Joint limits are checked by the caller.
  bool isValid(const moveit::core::RobotState& state) const;
  // The first contact, for messages.
  std::string describeContact(const moveit::core::RobotState& state) const;
  void message(const moveit::core::RobotState& state, moveit_msgs::PlanningScene& out) const;

  // The robot at its default configuration, the fixed joints set and the object attached.
  const moveit::core::RobotState& defaultState() const { return scene_->getCurrentState(); }
  const collision_detection::AllowedCollisionMatrix& allowedCollisions() const { return scene_->getAllowedCollisionMatrix(); }
  // The obstacles of the metric, in a fixed order.
  const std::vector<Box>& obstacles() const { return obstacles_; }

private:
  void addBox(const std::string& id, const Box& box);
  void addSceneFile(const std::string& file);
  void addWorldBoxLinks(const std::vector<std::string>& links);
  void attach(const CarriedSpec& carried);
  void allowRigidContacts(const ScenarioSpec& spec);

  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningScenePtr scene_;
  std::vector<Box> obstacles_;
};

}  // namespace robot
}  // namespace rmrrt

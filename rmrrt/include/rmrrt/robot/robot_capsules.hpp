#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <moveit/robot_model/robot_model.h>

#include "rmrrt/robot/scenario.hpp"

namespace rmrrt {
namespace robot {

// The capsules of the robot links in their link frames, never thicker than the collision shapes:
// a cylinder is the core of a capsule, a sphere not already inside one is a capsule of zero length, and a
// box is filled with parallel capsules plus its 12 edges. A robot with self-collision links (*_sc) uses
// only those. Links listed in fitted_capsules use those instead.
std::vector<std::pair<std::string, Capsule>> linkCapsules(const moveit::core::RobotModel& model, const std::vector<std::string>& world_box_links,
                                                          const std::map<std::string, std::vector<Capsule>>& fitted_capsules);

// The capsules of the carried object in the frame of its link: a thin box as the edges and midlines of
// its largest face, another box as its 12 edges and face midlines, a cylinder as one capsule, a mesh as
// its fitted capsules.
std::vector<Capsule> carriedCapsules(const CarriedSpec& spec);

}  // namespace robot
}  // namespace rmrrt

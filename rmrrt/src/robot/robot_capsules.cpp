#include "rmrrt/robot/robot_capsules.hpp"

#include <algorithm>
#include <cmath>

#include <urdf_model/model.h>

namespace rmrrt {
namespace robot {

namespace {

bool endsWith(const std::string& text, const std::string& suffix)
{
  return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

Eigen::Isometry3d toIsometry(const urdf::Pose& origin)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(origin.position.x, origin.position.y, origin.position.z);
  pose.linear() = Eigen::Quaterniond(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z).toRotationMatrix();
  return pose;
}

}  // namespace

std::vector<std::pair<std::string, Capsule>> linkCapsules(const moveit::core::RobotModel& model, const std::vector<std::string>& world_box_links,
                                                          const std::map<std::string, std::vector<Capsule>>& fitted_capsules)
{
  const auto urdf = model.getURDF();
  bool has_sc = false;
  for (const auto& entry : urdf->links_)
    has_sc = has_sc || endsWith(entry.first, "_sc");

  std::vector<std::pair<std::string, Capsule>> capsules;
  for (const auto& entry : urdf->links_)
  {
    const std::string& name = entry.first;
    if (has_sc && !endsWith(name, "_sc"))
      continue;

    std::vector<Capsule> on_link;
    const auto fitted = fitted_capsules.find(name);
    if (fitted != fitted_capsules.end())
    {
      for (const Capsule& capsule : fitted->second)
        capsules.emplace_back(name, capsule);
      continue;
    }
    // Never thicker than the collision shape: validity checks these very shapes, and a proxy that sticks
    // out makes the metric see contact where there is none.
    const auto endSphere = [&](const Eigen::Vector3d& point, double radius) {
      for (const auto& collision : entry.second->collision_array)
        if (collision->geometry && collision->geometry->type == urdf::Geometry::SPHERE &&
            (toIsometry(collision->origin).translation() - point).norm() < 2e-3 &&
            static_cast<const urdf::Sphere*>(collision->geometry.get())->radius > radius - 2e-3)
          return true;
      return false;
    };
    for (const auto& collision : entry.second->collision_array)
    {
      if (!collision->geometry)
        continue;
      const Eigen::Isometry3d pose = toIsometry(collision->origin);
      if (collision->geometry->type == urdf::Geometry::CYLINDER)
      {
        // With a sphere on each end (a capsule written as three shapes) the capsule is exact. A long
        // cylinder otherwise gets its ends set in by the radius; a flat one (a disc) is filled with
        // parallel chords of radius half its length, at most that far apart.
        const auto* cylinder = static_cast<const urdf::Cylinder*>(collision->geometry.get());
        const double r = cylinder->radius, half = 0.5 * cylinder->length;
        const bool capped = endSphere(pose * Eigen::Vector3d(0.0, 0.0, half), r) && endSphere(pose * Eigen::Vector3d(0.0, 0.0, -half), r);
        if (capped || half >= r)
        {
          const double inset = capped ? 0.0 : r;
          Capsule capsule;
          capsule.a = pose * Eigen::Vector3d(0.0, 0.0, -half + inset);
          capsule.b = pose * Eigen::Vector3d(0.0, 0.0, half - inset);
          capsule.radius = r;
          on_link.push_back(capsule);
        }
        else
        {
          const double disc = r - half;  // radius left for the chord centers
          const int chords = static_cast<int>(std::ceil(2.0 * disc / half)) + 1;
          for (int k = 0; k < chords; ++k)
          {
            const double y = -disc + 2.0 * disc * k / (chords - 1);
            const double x = std::sqrt(std::max(disc * disc - y * y, 0.0));
            Capsule chord;
            chord.a = pose * Eigen::Vector3d(-x, y, 0.0);
            chord.b = pose * Eigen::Vector3d(x, y, 0.0);
            chord.radius = half;
            on_link.push_back(chord);
          }
        }
      }
      else if (!has_sc && collision->geometry->type == urdf::Geometry::BOX &&
               std::find(world_box_links.begin(), world_box_links.end(), name) == world_box_links.end())
      {
        // Parallel capsules of radius r (half the thinnest side) along the longest side, spaced at
        // most r apart across the middle side, all set in by r: they fill the box without leaving it.
        // The 12 edges, as segments of radius 0, add what the rounded capsules miss: the corners.
        const auto* box = static_cast<const urdf::Box*>(collision->geometry.get());
        const Eigen::Vector3d size(box->dim.x, box->dim.y, box->dim.z);
        int thin = 0, longest = 0;
        size.minCoeff(&thin);
        size.maxCoeff(&longest);
        if (thin == longest)
          longest = (thin + 1) % 3;
        const int middle = 3 - thin - longest;
        const double r = 0.5 * size[thin];
        const double reach = 0.5 * size[longest] - r, span = 0.5 * size[middle] - r;
        const int lines = span > 1e-9 ? static_cast<int>(std::ceil(2.0 * span / r)) + 1 : 1;
        for (int k = 0; k < lines; ++k)
        {
          Eigen::Vector3d offset = Eigen::Vector3d::Zero(), along = Eigen::Vector3d::Zero();
          offset[middle] = lines > 1 ? -span + 2.0 * span * k / (lines - 1) : 0.0;
          along[longest] = reach;
          Capsule line;
          line.a = pose * (offset - along);
          line.b = pose * (offset + along);
          line.radius = r;
          on_link.push_back(line);
        }
        for (int axis = 0; axis < 3; ++axis)
          for (const double s1 : {-0.5, 0.5})
            for (const double s2 : {-0.5, 0.5})
            {
              Eigen::Vector3d corner;
              corner[axis] = -0.5 * size[axis];
              corner[(axis + 1) % 3] = s1 * size[(axis + 1) % 3];
              corner[(axis + 2) % 3] = s2 * size[(axis + 2) % 3];
              Eigen::Vector3d end = corner;
              end[axis] = 0.5 * size[axis];
              Capsule edge;
              edge.a = pose * corner;
              edge.b = pose * end;
              edge.radius = 0.0;
              on_link.push_back(edge);
            }
      }
    }

    // The ends of a cylinder are usually listed as spheres too; those add nothing.
    for (const auto& collision : entry.second->collision_array)
    {
      if (!collision->geometry || collision->geometry->type != urdf::Geometry::SPHERE)
        continue;
      const auto* sphere = static_cast<const urdf::Sphere*>(collision->geometry.get());
      const Eigen::Vector3d center = toIsometry(collision->origin).translation();
      bool contained = false;
      for (const Capsule& capsule : on_link)
      {
        const Eigen::Vector3d axis = capsule.b - capsule.a;
        const double t = axis.squaredNorm() > 1e-12 ? std::min(std::max((center - capsule.a).dot(axis) / axis.squaredNorm(), 0.0), 1.0) : 0.0;
        contained = contained || (center - (capsule.a + t * axis)).norm() + sphere->radius <= capsule.radius + 2e-3;
      }
      if (contained)
        continue;
      Capsule capsule;
      capsule.a = capsule.b = center;
      capsule.radius = sphere->radius;
      on_link.push_back(capsule);
    }

    for (const Capsule& capsule : on_link)
      capsules.emplace_back(name, capsule);
  }
  return capsules;
}

std::vector<Capsule> carriedCapsules(const CarriedSpec& spec)
{
  std::vector<Capsule> capsules;
  const auto add = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b, double radius) {
    Capsule capsule;
    capsule.a = spec.pose * a;
    capsule.b = spec.pose * b;
    capsule.radius = radius;
    capsules.push_back(capsule);
  };

  if (spec.shape == CarriedSpec::Shape::Mesh)
  {
    for (const Capsule& capsule : spec.mesh_capsules)
      add(capsule.a, capsule.b, capsule.radius);
    return capsules;
  }
  if (spec.shape == CarriedSpec::Shape::Cylinder)
  {
    add(Eigen::Vector3d(0.0, 0.0, -0.5 * spec.length), Eigen::Vector3d(0.0, 0.0, 0.5 * spec.length), spec.radius);
    return capsules;
  }

  int thin = 0;
  spec.size.minCoeff(&thin);
  const int u = (thin + 1) % 3, v = (thin + 2) % 3;
  if (spec.size[thin] >= 0.5 * std::min(spec.size[u], spec.size[v]))
  {
    const double radius = std::min(0.02, 0.25 * spec.size.minCoeff());
    const Eigen::Vector3d half = 0.5 * spec.size - Eigen::Vector3d::Constant(radius);
    for (int axis = 0; axis < 3; ++axis)
    {
      const int i = (axis + 1) % 3, j = (axis + 2) % 3;
      Eigen::Vector3d along = Eigen::Vector3d::Zero();
      along[axis] = half[axis];
      for (double si : {-1.0, 1.0})
        for (double sj : {-1.0, 1.0})
        {
          Eigen::Vector3d offset = Eigen::Vector3d::Zero();  // an edge along this axis
          offset[i] = si * half[i];
          offset[j] = sj * half[j];
          add(offset - along, offset + along, radius);
        }
      for (double sign : {-1.0, 1.0})
      {
        // The two faces normal to this axis: their midlines along i and j.
        Eigen::Vector3d face = Eigen::Vector3d::Zero(), di = Eigen::Vector3d::Zero(), dj = Eigen::Vector3d::Zero();
        face[axis] = sign * half[axis];
        di[i] = half[i];
        dj[j] = half[j];
        add(face - di, face + di, radius);
        add(face - dj, face + dj, radius);
      }
    }
    return capsules;
  }
  Eigen::Vector3d eu = Eigen::Vector3d::Zero(), ev = Eigen::Vector3d::Zero();
  eu[u] = 0.5 * spec.size[u];
  ev[v] = 0.5 * spec.size[v];
  const double radius = 0.5 * spec.size[thin];
  add(-eu - ev, eu - ev, radius);
  add(eu - ev, eu + ev, radius);
  add(eu + ev, -eu + ev, radius);
  add(-eu + ev, -eu - ev, radius);
  add(-eu, eu, radius);
  add(-ev, ev, radius);
  return capsules;
}

}  // namespace robot
}  // namespace rmrrt

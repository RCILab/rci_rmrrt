#include "rmrrt/planner/method.hpp"

#include <stdexcept>

#include "rmrrt/selection/selection_factory.hpp"
#include "rmrrt/steering/steering_factory.hpp"

namespace rmrrt {

const std::vector<Method>& methods()
{
  static const std::vector<Method> list{{"no_metric", "euclidean", "nearest"},
                                        {"steer", "centered", "nearest"},
                                        {"select", "euclidean", "riemannian"},
                                        {"rmrrt", "centered", "riemannian"}};
  return list;
}

const Method& findMethod(const std::string& name)
{
  for (const Method& method : methods())
    if (method.name == name)
      return method;
  throw std::invalid_argument("Unknown method " + name + " (no_metric | steer | select | rmrrt)");
}

void configure(RRTConnect& planner, const Method& method, double beta)
{
  const bool centered = method.steering == "centered" && beta > 0.0;
  planner.setSteering(createSteering(method.steering == "centered" && !centered ? "metric" : method.steering, beta));
  planner.setFallback(centered ? createSteering("metric") : nullptr);
  planner.setSelection(createSelection(method.selection, beta));
}

}  // namespace rmrrt

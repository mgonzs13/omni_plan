// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef OMNI_PLAN_MRTA__TASK_ALLOCATOR_HPP_
#define OMNI_PLAN_MRTA__TASK_ALLOCATOR_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/utils/parameter_loader.hpp"

namespace omni_plan_mrta {

/**
 * @struct RobotAllocation
 * @brief Goal indices assigned to one robot.
 */
struct RobotAllocation {
  /// Indices into the goals vector for this robot's sub-problem.
  std::vector<int> goal_indices;
};

/**
 * @class TaskAllocator
 * @brief Abstract base class for multi-robot task allocation strategies.
 *
 * @details Concrete implementations provide different algorithms for
 * distributing PDDL goal predicates among robots. Each implementation is
 * loaded as a pluginlib plugin by the MrtaPlanner at runtime.
 *
 * Derived classes may register ROS 2 parameters via
 * `add_ros_parameters()` in their constructor. Parameters are declared
 * and read under the namespace \c "planner.allocator".
 */
class TaskAllocator : public omni_plan::utils::ParameterLoader {
public:
  /**
   * @brief Constructor.
   *
   * Initializes the parameter loader with the namespace "allocator".
   * Derived classes can add their own parameters under this namespace.
   */
  TaskAllocator() : omni_plan::utils::ParameterLoader("allocator") {}

  /**
   * @brief Virtual destructor.
   */
  virtual ~TaskAllocator() = default;

  /**
   * @brief Allocates goals to robots.
   *
   * @param robots   Ordered list of robot names extracted from the problem.
   * @param goals    Ordered list of goal predicates from the problem.
   * @param problem  The full PDDL problem (objects + initial facts).
   * @param actions  Map of action name → Action template.
   * @return Map from robot name to its allocation (goal indices).
   */
  virtual std::unordered_map<std::string, RobotAllocation>
  allocate(const std::vector<std::string> &robots,
           const std::vector<omni_plan::pddl::Predicate> &goals,
           const omni_plan::pddl::Problem &problem,
           const std::unordered_map<std::string,
                                    std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const = 0;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__TASK_ALLOCATOR_HPP_

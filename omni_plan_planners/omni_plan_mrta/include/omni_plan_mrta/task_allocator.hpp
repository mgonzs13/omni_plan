// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OMNI_PLAN_MRTA__TASK_ALLOCATOR_HPP_
#define OMNI_PLAN_MRTA__TASK_ALLOCATOR_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/utils/parameter_loader.hpp"

namespace omni_plan_mrta {

/**
 * @struct TeamAllocation
 * @brief A planning group: one or more robots that jointly solve a set of
 * goals.
 *
 * @details When @c robots contains a single name this behaves identically to
 * the old per-robot allocation. When @c robots contains multiple names the
 * MrtaPlanner creates a shared sub-problem that includes all team members and
 * hands it to the sub-planner so it can produce a coordinated plan.
 */
struct TeamAllocation {
  /// Ordered list of robot names that form this planning group (≥ 1).
  std::vector<std::string> robots;
  /// Indices into the goals vector that are assigned to this group.
  std::vector<int> goal_indices;
};

/**
 * @class TaskAllocator
 * @brief Abstract base class for multi-robot task allocation strategies.
 *
 * @details Concrete implementations provide different algorithms for
 * distributing PDDL goal predicates among robots, optionally forming robot
 * teams for goals that require multi-robot cooperation.  Each implementation
 * is loaded as a pluginlib plugin by the MrtaPlanner at runtime.
 *
 * The returned @c vector<TeamAllocation> partitions all goals into planning
 * groups.  Single-robot groups reproduce the classic individual allocation;
 * multi-robot groups allow the sub-planner to produce coordinated plans.
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
   * @brief Allocates goals to robot planning groups (teams).
   *
   * @param robots   Ordered list of robot names extracted from the problem.
   * @param goals    Ordered list of goal predicates from the problem.
   * @param problem  The full PDDL problem (objects + initial facts).
   * @param actions  Map of action name → Action template.
   * @return List of TeamAllocation entries partitioning all goals. Each entry
   *         has one or more robots and the goal indices for that group.
   */
  virtual std::vector<TeamAllocation>
  allocate(const std::vector<std::string> &robots,
           const std::vector<omni_plan::pddl::Predicate> &goals,
           const omni_plan::pddl::Problem &problem,
           const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const = 0;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__TASK_ALLOCATOR_HPP_

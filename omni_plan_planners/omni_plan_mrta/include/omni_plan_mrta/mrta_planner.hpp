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

#ifndef OMNI_PLAN_MRTA__MRTA_PLANNER_HPP_
#define OMNI_PLAN_MRTA__MRTA_PLANNER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yasmin_ros/yasmin_node.hpp"

#include "omni_plan/planner.hpp"
#include "omni_plan_mrta/task_allocator.hpp"

namespace omni_plan_mrta {

/**
 * @class MrtaPlanner
 * @brief Multi-Robot Task Allocation planner that decomposes PDDL problems
 * across robots using a pluggable task allocator and a single sub-planner.
 *
 * @details For multi-robot problems, this planner:
 *   1. Identifies robots from the PDDL problem objects (by type).
 *   2. Delegates goal allocation to a TaskAllocator plugin.
 *   3. Creates per-robot sub-problems.
 *   4. Solves each sub-problem in parallel with the configured sub-planner.
 *   5. Merges the resulting sub-plans sorted by action start time.
 *
 * For single-robot problems (or problems with no robot objects), the
 * sub-planner is invoked directly on the original problem.
 *
 * ROS 2 parameters (under the @c planner namespace):
 *   - @c robot_type        (string, default "robot") — PDDL type for robots.
 *   - @c planner_plugin    (string) — pluginlib class of the sub-planner.
 *   - @c allocator_plugin  (string) — pluginlib class of the task allocator.
 *
 * Sub-planner parameters are loaded from the @c planner.sub_planner namespace.
 * Allocator parameters are loaded from the @c planner.allocator namespace.
 */
class MrtaPlanner : public omni_plan::Planner {
public:
  MrtaPlanner();

  omni_plan::pddl::Plan
  generate_plan(const omni_plan::pddl::Domain &domain,
                const omni_plan::pddl::Problem &problem) const override;

  omni_plan::pddl::Plan parse_plan(const omni_plan::pddl::Domain &domain,
                                   const std::string &str_plan) const override;

private:
  /// @brief PDDL type name that identifies robot objects (e.g., "robot")
  std::string robot_type_;
  /// @brief Plugin class name for the sub-planner (e.g.,
  /// "omni_plan_popf/PopfPlanner")
  std::string planner_plugin_;
  /// @brief Plugin class name for the task allocator
  std::string allocator_plugin_;

  /// Node used for logging and parameter loading
  mutable std::shared_ptr<rclcpp::Node> node_;

  /// @brief pluginlib class loader for omni_plan::Planner plugins
  mutable std::unique_ptr<pluginlib::ClassLoader<omni_plan::Planner>>
      planner_loader_;
  /// @brief pluginlib class loader for TaskAllocator plugins
  mutable std::unique_ptr<pluginlib::ClassLoader<TaskAllocator>>
      allocator_loader_;

  /// @brief Loaded sub-planner plugin instance
  mutable std::shared_ptr<omni_plan::Planner> sub_planner_;
  /// @brief Loaded task allocator plugin instance
  mutable std::shared_ptr<TaskAllocator> allocator_;

  /**
   * @brief Extracts robot names from the PDDL problem objects section.
   * @param problem The PDDL problem from which to extract robot names.
   * @return A vector of robot names.
   */
  std::vector<std::string>
  extract_robots(const omni_plan::pddl::Problem &problem) const;

  /**
   * @brief Allocates goals to robots.
   * @param robots The list of robot names.
   * @param goals The list of goals to allocate.
   * @param problem The PDDL problem.
   * @param actions The map of available actions.
   * @return A vector of allocated goals for each robot.
   */
  std::vector<omni_plan_mrta::TeamAllocation> allocate_goals(
      const std::vector<std::string> &robots,
      const std::vector<omni_plan::pddl::Predicate> &goals,
      const omni_plan::pddl::Problem &problem,
      const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
          &actions) const;

  /**
   * @brief Merges multiple sub-plans into a single temporally-sorted plan.
   * @param plans The list of sub-plans to merge.
   * @return The merged plan.
   */
  omni_plan::pddl::Plan
  merge_plans(const std::vector<omni_plan::pddl::Plan> &plans) const;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__MRTA_PLANNER_HPP_

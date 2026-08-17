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

/**
 * @file homeostatic_planner.hpp
 * @brief Homeostatic planner with POIROT-based cost-aware planner selection.
 *
 * Extends CachePlanner so that exact and structural caching are applied
 * transparently.  On a cache miss, a UCB1 bandit selects among
 * multiple sub-planners to solve the problem, and the call is profiled
 * with POIROT to guide future selections.
 */

#ifndef OMNI_PLAN_HOMEOSTATIC__HOMEOSTATIC_PLANNER_HPP_
#define OMNI_PLAN_HOMEOSTATIC__HOMEOSTATIC_PLANNER_HPP_

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "omni_plan_cache/cache_planner.hpp"

#include "poirot_msgs/msg/profiling_data.hpp"

#include "omni_plan_homeostatic/homeostatic_planner_selector.hpp"

namespace omni_plan_homeostatic {

/**
 * @brief Homeostatic planner that adds UCB1 planner selection and
 *        POIROT profiling on top of CachePlanner's two-level caching.
 *
 * Inherits exact and structural caching from CachePlanner.  On a cache miss
 * the overridden delegate_plan selects a sub-planner via a UCB1
 * bandit (see HomeostaticPlannerSelector), profiles the call with POIROT,
 * and records the observed cost per problem hash.  Only successful plans
 * are cached (should_cache_result returns plan.has_solution()).
 */
class HomeostaticPlanner : public omni_plan_cache::CachePlanner {
public:
  /** @brief Constructor.  Registers ROS parameters and obtains the YasminNode.
   */
  HomeostaticPlanner();

  /** @brief Default destructor. */
  ~HomeostaticPlanner() override = default;

protected:
  /**
   * @brief Called by CachePlanner on cache miss to produce a plan.
   *
   * Selects a sub-planner via the UCB1 bandit for the given
   * structural_key (pre-computed by the parent), profiles it with POIROT,
   * records the observed cost, and returns the resulting plan.
   *
   * @param domain          The PDDL domain.
   * @param problem         The PDDL problem.
   * @param structural_key  The role-aware hash pre-computed by CachePlanner.
   * @param out_source_planner  Optional out-param filled with the planner
   *                            that produced the plan (used to parse the
   *                            cached output on later structural hits).
   * @return The plan produced by the selected sub-planner and the planner
   * itself.
   */
  omni_plan::pddl::Plan
  delegate_plan(const omni_plan::pddl::Domain &domain,
                const omni_plan::pddl::Problem &problem,
                const std::string &structural_key) const override;

  /**
   * @brief Only cache successful plans so that failed attempts do not
   *        prevent the bandit from trying other planners.
   *
   * @param plan The plan just produced by delegate_plan.
   * @return true iff plan.has_solution() is true.
   */
  bool should_cache_result(const omni_plan::pddl::Plan &plan) const override;

private:
  /** @brief List of planner short names to load (e.g. "popf_planner"). */
  std::vector<std::string> planner_plugins_;
  /** @brief UCB1 exploration constant. */
  double ucb_exploration_constant_;
  /** @brief Number of cold-start ensemble steps. */
  int cold_start_steps_;
  /** @brief POIROT Data field to use as cost (e.g. "total_energy_uj"). */
  std::string selection_field_;
  /** @brief Whether plan caching is enabled (can be toggled at runtime). */
  bool enable_cache_;

  /** @brief UCB1 bandit that holds sub-planners and cost history. */
  mutable std::shared_ptr<HomeostaticPlannerSelector> selector_;

  /** @brief Subscription to /poirot/data for profiling results. */
  mutable rclcpp::Subscription<poirot_msgs::msg::ProfilingData>::SharedPtr
      poirot_sub_;
  /**
   * @brief POIROT results keyed by unique profiler name.
   *
   * Populated by the subscription callback on the YasminNode executor
   * thread and consumed by call_sub_planner on the calling thread.
   */
  mutable std::map<std::string, poirot_msgs::msg::Data> poirot_results_;
  /** @brief Mutex protecting poirot_results_. */
  mutable std::mutex poirot_results_mutex_;
  /** @brief Condition variable signalled when a new result is stored. */
  mutable std::condition_variable poirot_cv_;
  /** @brief Monotonic counter appended to profiler names for uniqueness. */
  mutable std::atomic<size_t> call_seq_{0};

  /**
   * @brief Extract the configured cost field from a POIROT Data message.
   *
   * Looks up this->selection_field_ and returns the corresponding field.
   * Falls back to total_energy_uj for unknown field names.
   *
   * @param data The POIROT Data message.
   * @return The numeric value of the selected field.
   */
  double get_field_from_data(const poirot_msgs::msg::Data &data) const;

  /**
   * @brief Profile and execute a sub-planner, then retrieve its POIROT cost.
   *
   * Wraps planner->generate_plan() with POIROT start/stop using a unique
   * profiler name (built from planner_name and call_seq_), then waits on a
   * condition variable for the subscription callback to store the result.
   *
   * @param planner_name  Human-readable name for the profiler (e.g. "POPF").
   * @param planner       The sub-planner instance to call.
   * @param domain        The PDDL domain.
   * @param problem       The PDDL problem.
   * @return A pair of the generated plan and the observed cost.
   */
  std::pair<omni_plan::pddl::Plan, double>
  call_sub_planner(const std::string &planner_name,
                   std::shared_ptr<omni_plan::Planner> planner,
                   const omni_plan::pddl::Domain &domain,
                   const omni_plan::pddl::Problem &problem) const;
};

} // namespace omni_plan_homeostatic

#endif // OMNI_PLAN_HOMEOSTATIC__HOMEOSTATIC_PLANNER_HPP_

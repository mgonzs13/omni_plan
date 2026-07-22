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
 * Wraps multiple planner plugins behind an epsilon-greedy bandit that uses
 * real profiling data (energy, time, CO2, etc.) from POIROT to select the
 * cheapest planner for each problem hash.
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

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/planner.hpp"

#include "poirot_msgs/msg/profiling_data.hpp"

#include "yasmin_ros/yasmin_node.hpp"

#include "omni_plan_homeostatic/homeostatic_planner_selector.hpp"

namespace omni_plan_homeostatic {

/**
 * @brief Homeostatic planner plugin that wraps sub-planners and selects
 *        between them based on observed POIROT profiling cost.
 *
 * Each call to generate_plan selects a sub-planner via an epsilon-greedy
 * bandit (see HomeostaticPlannerSelector), profiles the call with POIROT
 * under a unique function name, and records the observed cost per problem
 * hash.  The subscription on /poirot/data captures the profiling result
 * keyed by the unique function name so parallel calls do not interfere.
 */
class HomeostaticPlanner : public omni_plan::Planner {
public:
  /** @brief Constructor.  Registers ROS parameters and obtains the YasminNode.
   */
  HomeostaticPlanner();

  /** @brief Default destructor. */
  ~HomeostaticPlanner() override = default;

  /**
   * @brief Select a sub-planner, profile it, and return the plan.
   *
   * The observed POIROT cost is recorded in the selector so future calls
   * can make better decisions for the same problem hash.
   *
   * @param domain  The PDDL domain.
   * @param problem The PDDL problem.
   * @return The plan produced by the selected sub-planner.
   */
  omni_plan::pddl::Plan
  generate_plan(const omni_plan::pddl::Domain &domain,
                const omni_plan::pddl::Problem &problem) const override;

  using Planner::generate_plan;

private:
  /** @brief List of planner short names to load (e.g. "popf_planner"). */
  std::vector<std::string> planner_plugins_;
  /** @brief Initial epsilon for the bandit (fraction of exploration). */
  double exploration_prob_;
  /** @brief Per-10-calls decay factor applied to epsilon. */
  double decay_rate_;
  /** @brief Floor for the decayed epsilon. */
  double min_exploration_;
  /** @brief POIROT Data field to use as cost (e.g. "total_energy_uj"). */
  std::string selection_field_;

  /** @brief YasminNode singleton for subscriptions and logging. */
  mutable yasmin_ros::YasminNode::SharedPtr yasmin_node_;
  /** @brief Plugin loader for sub-planner instances. */
  mutable pluginlib::ClassLoader<omni_plan::Planner> planner_loader_;
  /** @brief Epsilon-greedy bandit that holds sub-planners and cost history. */
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
   * @brief Produce a deterministic hash from a PDDL domain+problem pair.
   *
   * Object names are replaced by type-indexed placeholders so that
   * structurally identical problems with different object names map to the
   * same hash.  Uses SHA-256.
   *
   * @param domain  The PDDL domain.
   * @param problem The PDDL problem.
   * @return Hex-encoded SHA-256 hash.
   */
  std::string
  compute_problem_hash(const omni_plan::pddl::Domain &domain,
                       const omni_plan::pddl::Problem &problem) const;

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

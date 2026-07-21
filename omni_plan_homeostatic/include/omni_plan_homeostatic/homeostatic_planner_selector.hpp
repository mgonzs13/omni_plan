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
 * @file homeostatic_planner_selector.hpp
 * @brief Epsilon-greedy bandit for cost-aware planner selection.
 *
 * Maintains a per-problem-hash cost history for each registered planner
 * and selects the cheapest known planner for a given hash (exploitation)
 * or a random planner (exploration) with decaying probability.
 */

#ifndef OMNI_PLAN_HOMEOSTATIC__HOMEOSTATIC_PLANNER_SELECTOR_HPP_
#define OMNI_PLAN_HOMEOSTATIC__HOMEOSTATIC_PLANNER_SELECTOR_HPP_

#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/planner.hpp"

namespace omni_plan_homeostatic {

/**
 * @brief Per-planner accumulated cost and success statistics.
 */
struct PlannerCostRecord {
  /** @brief Name of the planner (matches the key in planners_). */
  std::string planner_name;
  /** @brief Sum of observed costs across all selections. */
  double total_cost;
  /** @brief Number of times this planner was selected. */
  size_t times_selected;
  /** @brief Number of times this planner returned a valid plan. */
  size_t times_succeeded;
};

/**
 * @brief Epsilon-greedy multi-armed bandit for planner selection.
 *
 * For each problem hash the selector tracks the average cost of every
 * planner that has been tried.  On select_planner the decision is:
 *
 *   - With probability epsilon: pick a random planner (exploration).
 *   - Otherwise: pick the planner with the lowest average cost for this
 *     hash (exploitation).  If no data exists for this hash, fall back
 *     to the planner with the lowest global average across all hashes.
 *
 * Epsilon decays exponentially every 10 calls and is floored at
 * min_exploration to guarantee ongoing exploration.
 */
class HomeostaticPlannerSelector {
public:
  /**
   * @brief Construct the bandit with the given exploration schedule.
   *
   * @param exploration_prob  Initial exploration probability (default 0.3).
   * @param decay_rate        Multiplicative decay every 10 calls (default
   * 0.95).
   * @param min_exploration   Floor for the decayed probability (default 0.05).
   */
  HomeostaticPlannerSelector(double exploration_prob = 0.3,
                             double decay_rate = 0.95,
                             double min_exploration = 0.05);

  /**
   * @brief Register a planner instance.
   *
   * @param name    Key used to identify this planner in the cost table.
   * @param planner Shared pointer to the planner plugin instance.
   */
  void add_planner(const std::string &name,
                   std::shared_ptr<omni_plan::Planner> planner);

  /**
   * @brief Select a planner for the given problem hash.
   *
   * Thread-safe (locked via selector_mutex_).
   *
   * @param hash_key          Deterministic hash of the problem.
   * @param out_planner_name  [out] Filled with the selected planner name.
   * @return The selected planner instance.
   */
  std::shared_ptr<omni_plan::Planner>
  select_planner(const std::string &hash_key, std::string &out_planner_name);

  /**
   * @brief Record the outcome of a planning attempt.
   *
   * Thread-safe (locked via selector_mutex_).
   *
   * @param hash_key      Problem hash the planner was invoked on.
   * @param planner_name  Name of the planner that was used.
   * @param cost          Observed cost (from POIROT or fallback).
   * @param succeeded     Whether a valid plan was produced.
   */
  void record_observation(const std::string &hash_key,
                          const std::string &planner_name, double cost,
                          bool succeeded);

  /**
   * @brief Produce a human-readable summary of the cost table.
   *
   * Thread-safe (locked via selector_mutex_).
   *
   * @return Multi-line string with per-hash and per-planner stats.
   */
  std::string get_planner_cost_table() const;

  /**
   * @brief Return the number of registered planners.
   *
   * @return Number of planners.
   */
  size_t get_num_planners() const { return planners_.size(); }

private:
  /** @brief Initial exploration probability. */
  double exploration_prob_;
  /** @brief Decay rate applied every 10 calls. */
  double decay_rate_;
  /** @brief Minimum exploration probability after decay. */
  double min_exploration_;
  /** @brief Total number of select_planner calls (for decay calculation). */
  size_t total_calls_;

  /** @brief Map of planner name to planner instance. */
  std::unordered_map<std::string, std::shared_ptr<omni_plan::Planner>>
      planners_;
  /**
   * @brief Per-hash, per-planner cost records.
   *
   *   cost_table_[hash][planner_name] == PlannerCostRecord
   */
  std::unordered_map<std::string,
                     std::unordered_map<std::string, PlannerCostRecord>>
      cost_table_;

  /** @brief Mutex protecting select_planner, record_observation, and the
   *         cost table. */
  mutable std::mutex selector_mutex_;
  /** @brief Mersenne Twister RNG for random exploration picks. */
  std::mt19937 rng_;
};

} // namespace omni_plan_homeostatic

#endif // OMNI_PLAN__HOMEOSTATIC_PLANNER_SELECTOR_HPP_

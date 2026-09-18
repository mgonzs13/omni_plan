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
 * @file component_composer.hpp
 * @brief Goal decomposition and sub-plan composition used to solve recurrent
 * sub-problems through the cache.
 */

#ifndef OMNI_PLAN_CACHE__DETAIL__COMPONENT_COMPOSER_HPP_
#define OMNI_PLAN_CACHE__DETAIL__COMPONENT_COMPOSER_HPP_

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <cstddef>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"

namespace omni_plan_cache {
namespace detail {

/**
 * @class ComponentComposer
 * @brief Decomposes goals into independent components and stitches sub-plans.
 * @details Goals that share an object form one component. Each component is
 * solved as a small standalone problem (highly cache-friendly because
 * component structures recur constantly); the sub-plans are stitched together
 * in sequence, with effects simulated between components so later components
 * see earlier results. Components whose mutable objects are pairwise disjoint
 * may be solved concurrently. The caller is responsible for validating the
 * composed plan against the full problem before using it.
 */
class ComponentComposer {
public:
  /**
   * @struct Options
   * @brief Tuning knobs for decomposition and solving.
   */
  struct Options {
    /// @brief Object type always seeded into every component sub-problem
    /// (typically robots).
    std::string robot_type = "robot";
    /// @brief Goal predicate whose component is ordered first (for example a
    /// battery predicate); an empty string disables prioritization.
    std::string priority_predicate;
    /// @brief Components with more goals than this are not composed.
    size_t max_goals_per_component = 4;
    /// @brief Allow concurrent solving when mutable objects are disjoint.
    bool parallel_independent = true;
  };

  /// @brief Callback used to solve one component sub-problem.
  using Solver = std::function<omni_plan::pddl::Plan(
      const omni_plan::pddl::Domain &, const omni_plan::pddl::Problem &)>;
  /// @brief Callback used to report informational or warning messages.
  using Logger = std::function<void(const std::string &)>;

  /**
   * @brief Creates a composer bound to a solver and optional loggers.
   * @param options Decomposition and parallelism options.
   * @param solver Callback that solves one component sub-problem.
   * @param info_log Optional informational logger (may be empty).
   * @param warn_log Optional warning logger (may be empty).
   */
  ComponentComposer(Options options, Solver solver, Logger info_log = {},
                    Logger warn_log = {});

  /**
   * @brief Decomposes the problem goals and composes a full plan.
   * @details Returns false (without touching @p out_plan) when composition
   * does not apply: fewer than two components, a component exceeding
   * Options::max_goals_per_component, or any component without a solution.
   * On success, @p out_plan receives the ordered composition of the sub-plans
   * with sequential start times and has_solution set to true. The result is
   * NOT validated here.
   * @param domain The PDDL domain.
   * @param problem The full PDDL problem.
   * @param relevant_facts Relevant facts used as the initial simulated state.
   * @param full_static_predicates Domain predicates never modified by any
   * action; their facts are kept in every component sub-problem.
   * @param out_plan Output parameter filled on success.
   * @return true if a composed plan was produced, false otherwise.
   */
  bool compose(const omni_plan::pddl::Domain &domain,
               const omni_plan::pddl::Problem &problem,
               const std::set<omni_plan::pddl::Predicate> &relevant_facts,
               const std::set<std::string> &full_static_predicates,
               omni_plan::pddl::Plan &out_plan);

  /**
   * @brief Applies the instantiated effects of one action to a fact set.
   * @details Resolves each effect argument through the action's parameter
   * name-to-index map and the given plan parameters, then inserts or erases
   * the resulting predicate. Start effects are applied before end effects,
   * mirroring plan execution.
   * @param facts Fact set to update in place.
   * @param action Action whose effects are applied.
   * @param params Concrete parameters for this action instance.
   */
  static void
  apply_action_effects(std::set<omni_plan::pddl::Predicate> &facts,
                       const std::shared_ptr<omni_plan::pddl::Action> &action,
                       const std::vector<std::string> &params);

  /**
   * @brief Checks whether components can be solved concurrently.
   * @details Independent means the mutable-object sets are pairwise disjoint,
   * so no sub-plan can change the state another sub-plan depends on.
   * @param mutable_objects Mutable-object set of each component.
   * @return true when every pair of sets is disjoint.
   */
  static bool can_solve_in_parallel(
      const std::vector<std::set<std::string>> &mutable_objects);

private:
  /// @brief Decomposition and parallelism options.
  Options options_;
  /// @brief Solver invoked once per component sub-problem.
  Solver solver_;
  /// @brief Informational logger (may be empty).
  Logger info_;
  /// @brief Warning logger (may be empty).
  Logger warn_;
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__COMPONENT_COMPOSER_HPP_

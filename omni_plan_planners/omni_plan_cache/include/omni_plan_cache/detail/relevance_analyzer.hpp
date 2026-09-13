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
 * @file relevance_analyzer.hpp
 * @brief Goal-oriented relevance analysis used to stabilize structural cache
 * keys by excluding state that cannot influence the plan.
 */

#ifndef OMNI_PLAN_CACHE__DETAIL__RELEVANCE_ANALYZER_HPP_
#define OMNI_PLAN_CACHE__DETAIL__RELEVANCE_ANALYZER_HPP_

#include <set>
#include <string>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"

namespace omni_plan_cache {
namespace detail {

/**
 * @struct RelevanceResult
 * @brief Predicate, object and fact sets that are relevant to the goals.
 * @details Produced by RelevanceAnalyzer::analyze() and consumed by the
 * structural keyer and the component composer.
 */
struct RelevanceResult {
  /// @brief Predicate names reachable by backward chaining from the goals.
  std::set<std::string> relevant_predicates;
  /// @brief Relevant predicates that no action ever modifies (static state).
  std::set<std::string> static_predicates;
  /// @brief All domain predicates that no action ever modifies, whether
  /// relevant or not; used to retain the objects they mention.
  std::set<std::string> full_static_predicates;
  /// @brief Objects that can influence the plan: robots, goal arguments and
  /// arguments of relevant static facts.
  std::set<std::string> relevant_objects;
  /// @brief Initial facts kept for structural keying: relevant static facts,
  /// plus relevant facts that mention a relevant object.
  std::set<omni_plan::pddl::Predicate> relevant_facts;
};

/**
 * @class RelevanceAnalyzer
 * @brief Backward-chaining relevance analysis with a per-call action index.
 * @details Starting from the goal predicates, walks predicates through action
 * effects to the conditions that achieve them; every predicate reached is
 * relevant. Predicates that can never influence the goals are excluded from
 * structural keys so that changes in purely observational state do not cause
 * cache misses.
 */
class RelevanceAnalyzer {
public:
  /**
   * @brief Computes the set of predicates relevant to the goals.
   * @details Builds an effect-name to actions index and a per-action
   * condition-name list once, then runs a fixpoint backward chain from the
   * goal predicates.
   * @param domain The PDDL domain with action definitions.
   * @param problem The PDDL problem whose goals seed the analysis.
   * @return Names of all predicates that can influence goal achievement.
   */
  static std::set<std::string>
  relevant_predicates(const omni_plan::pddl::Domain &domain,
                      const omni_plan::pddl::Problem &problem);

  /**
   * @brief Runs the full relevance analysis for a problem.
   * @details Combines relevant_predicates() with the static-predicate split,
   * the relevant-object computation (robots of type @p robot_type, goal
   * arguments and static-fact arguments) and the filtered relevant-fact set.
   * @param domain The PDDL domain.
   * @param problem The PDDL problem.
   * @param robot_type Object type always considered relevant (robots); pass
   * an unknown type to disable the implicit robot inclusion.
   * @return All relevance sets required by the structural and component
   * pipeline.
   */
  static RelevanceResult analyze(const omni_plan::pddl::Domain &domain,
                                 const omni_plan::pddl::Problem &problem,
                                 const std::string &robot_type);
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__RELEVANCE_ANALYZER_HPP_

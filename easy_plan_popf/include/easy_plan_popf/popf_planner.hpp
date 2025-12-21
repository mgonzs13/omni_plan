// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#ifndef EASY_PLAN_POPF__POPF_PLANNER_HPP_
#define EASY_PLAN_POPF__POPF_PLANNER_HPP_

#include <memory>
#include <string>

#include "easy_plan/pddl/plan.hpp"
#include "easy_plan/planner.hpp"

namespace easy_plan_popf {

/**
 * @class PopfPlanner
 * @brief Planner implementation using the POPF (Partial Order Planning with
 * First-Order logic) algorithm.
 * @details This class provides automated planning capabilities using the POPF
 * algorithm, which is particularly effective for temporal planning domains with
 * durative actions. POPF can handle complex planning problems involving
 * concurrent actions and temporal constraints.
 */
class PopfPlanner : public easy_plan::Planner {
public:
  /**
   * @brief Default constructor for PopfPlanner.
   * @details Initializes the POPF planner with default settings and prepares
   * it for solving planning problems.
   */
  PopfPlanner();

  /**
   * @brief Generates a plan from PDDL domain and problem file paths.
   * @details This pure virtual method must be implemented by derived classes to
   * read PDDL domain and problem definitions from files, then generate a plan
   * to solve the problem.
   * @param domain_path The file path to the PDDL domain definition.
   * @param problem_path The file path to the PDDL problem definition.
   * @return A string representing the generated plan in PDDL format.
   */
  std::string generate_plan(const std::string domain_path,
                            const std::string problem_path) const override;

  using Planner::generate_plan;

  /**
   * @brief Checks if the generated plan indicates a valid solution.
   * @details This pure virtual method must be implemented by derived classes to
   * analyze the plan output and determine if it represents a valid solution to
   * the planning problem.
   * @param plan_str The generated plan as a string.
   * @return True if the plan represents a valid solution, false otherwise.
   */
  bool has_solution(const std::string &plan_output) const override;

private:
  /// @brief Disable best-first search; if EHC fails, abort.
  bool disable_best_first_;
  /// @brief Skip EHC: go straight to best-first search.
  bool skip_ehc_;
  /// @brief Use standard EHC instead of steepest descent.
  bool standard_ehc_;
  /// @brief Disable helpful-action pruning.
  bool disable_helpful_pruning_;
  /// @brief Disable compression-safe action detection.
  bool disable_compression_safe_;
  /// @brief Disable the tie-breaking in RPG that favour actions that slot into
  /// the partial order earlier.
  bool disable_tie_breaking_rpg_;
  /// @brief Sort initial layer facts in RPG by availability order (only use if
  /// using -c).
  bool sort_initial_layer_;
  /// @brief Disable the tie-breaking in search that favours plans with shorter
  /// makespans.
  bool disable_tie_breaking_search_;
  /// @brief Full FF helpful actions (rather than just those in the RP
  /// applicable in the current state).
  bool full_ff_helpful_;
  /// @brief Branch ordering so actions violating fewer other actions'
  /// preconditions come first.
  bool branch_ordering_;
  /// @brief Try to use better actions in the heuristic, using h-add costs.
  bool better_actions_heuristic_;
  /// @brief Disable the use of an STP in cases where it is sufficient.
  bool disable_stp_;
  /// @brief Rather than building a partial order, build a total-order.
  bool total_order_;
};

} // namespace easy_plan_popf
#endif // EASY_PLAN_POPF__POPF_PLANNER_HPP_
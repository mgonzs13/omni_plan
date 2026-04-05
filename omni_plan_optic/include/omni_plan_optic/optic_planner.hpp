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

#ifndef OMNI_PLAN_OPTIC__OPTIC_PLANNER_HPP_
#define OMNI_PLAN_OPTIC__OPTIC_PLANNER_HPP_

#include <string>
#include <vector>

#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/planner.hpp"

namespace omni_plan_optic {

/**
 * @class OpticPlanner
 * @brief Planner implementation using the OPTIC (Optimising Preferences and
 * Time-dependent Costs) algorithm.
 * @details This class provides automated planning capabilities using the OPTIC
 * algorithm, which extends POPF with support for preferences and
 * time-dependent costs. OPTIC can handle complex temporal planning problems
 * with concurrent actions, preferences, and metric optimisation.
 */
class OpticPlanner : public omni_plan::Planner {
public:
  /**
   * @brief Default constructor for OpticPlanner.
   * @details Initializes the OPTIC planner with default settings and prepares
   * it for solving planning problems.
   */
  OpticPlanner();

  /**
   * @brief Generates a plan from PDDL domain and problem file paths.
   * @param domain_path The file path to the PDDL domain definition.
   * @param problem_path The file path to the PDDL problem definition.
   * @return A string representing the generated plan output.
   */
  std::string generate_plan(const std::string domain_path,
                            const std::string problem_path) const override;

  using Planner::generate_plan;

  /**
   * @brief Checks if the generated plan output indicates a valid solution.
   * @param plan_output The complete planner output as a string.
   * @return True if the plan represents a valid solution, false otherwise.
   */
  bool has_solution(const std::string &plan_output) const override;

  /**
   * @brief Extracts lines containing actions from the OPTIC plan output.
   * @details OPTIC may print the plan twice (once during search and once as
   * the final solution). This override extracts only the lines that appear
   * after the final ";;;; Solution Found" marker.
   * @param plan_str The complete plan output as a string.
   * @return A vector of strings, each representing an action line.
   */
  std::vector<std::string>
  get_lines_with_actions(const std::string &plan_str) const override;

private:
  /// @brief Don't optimise solution quality (ignores preferences and costs).
  bool no_optimise_;
  /// @brief Abstract out timed initial literals that represent recurrent
  /// windows.
  bool abstract_timed_literals_;
  /// @brief Optimise solution quality, capping cost at this value (-1 to
  /// disable).
  double cost_limit_;
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
  /// @brief Enable tie-breaking in RPG that favours actions slotting into the
  /// partial order earlier.
  bool tie_breaking_rpg_;
  /// @brief Sort initial layer facts in RPG by availability order (only use
  /// with tie_breaking_rpg).
  bool sort_initial_layer_;
  /// @brief Disable tie-breaking in search that favours plans with shorter
  /// makespans.
  bool disable_tie_breaking_search_;
  /// @brief Full FF helpful actions (rather than just those in the RP
  /// applicable in the current state).
  bool full_ff_helpful_;
  /// @brief Rather than building a partial order, build a total-order.
  bool total_order_;
};

} // namespace omni_plan_optic
#endif // OMNI_PLAN_OPTIC__OPTIC_PLANNER_HPP_

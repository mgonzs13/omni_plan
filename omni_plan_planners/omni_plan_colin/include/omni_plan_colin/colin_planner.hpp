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

#ifndef OMNI_PLAN_COLIN__COLIN_PLANNER_HPP_
#define OMNI_PLAN_COLIN__COLIN_PLANNER_HPP_

#include <memory>
#include <string>

#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/planner.hpp"

namespace omni_plan_colin {

/**
 * @class ColinPlanner
 * @brief Planner implementation using the COLIN algorithm.
 * @details This class provides automated planning capabilities using the COLIN
 * (Colin Is Not POPF) algorithm, which extends POPF with support for
 * continuous numeric change. COLIN is particularly effective for temporal
 * planning domains with durative actions and numeric constraints.
 */
class ColinPlanner : public omni_plan::Planner {
public:
  /**
   * @brief Default constructor for ColinPlanner.
   * @details Initializes the COLIN planner with default settings and prepares
   * it for solving planning problems.
   */
  ColinPlanner();

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
  /// @brief Disable the tie-breaking in search that favours plans with shorter
  /// makespans.
  bool disable_tie_breaking_search_;
  /// @brief Full FF helpful actions (rather than just those in the RP
  /// applicable in the current state).
  bool full_ff_helpful_;
  /// @brief Rather than building a partial order, build a total-order.
  bool total_order_;
};

} // namespace omni_plan_colin

#endif // OMNI_PLAN_COLIN__COLIN_PLANNER_HPP_

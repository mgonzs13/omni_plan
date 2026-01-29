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

#ifndef OMNI_PLAN_VHPOP__VHPOP_PLANNER_HPP_
#define OMNI_PLAN_VHPOP__VHPOP_PLANNER_HPP_

#include <memory>
#include <string>

#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/planner.hpp"

namespace omni_plan_vhpop {

/**
 * @class VhpopPlanner
 * @brief Planner implementation using the VHPOP (Versatile Heuristic Partial
 * Order Planner) algorithm.
 * @details This class provides automated planning capabilities using the VHPOP
 * algorithm, which is particularly effective for temporal planning domains with
 * durative actions. VHPOP can handle complex planning problems involving
 * concurrent actions and temporal constraints.
 */
class VhpopPlanner : public omni_plan::Planner {
public:
  /**
   * @brief Default constructor for VhpopPlanner.
   * @details Initializes the VHPOP planner with default settings and prepares
   * it for solving planning problems.
   */
  VhpopPlanner();

  /**
   * @brief Generates a plan from PDDL domain and problem file paths.
   * @details Reads PDDL domain and problem definitions from files, then
   * generates a plan to solve the problem using the VHPOP algorithm.
   * @param domain_path The file path to the PDDL domain definition.
   * @param problem_path The file path to the PDDL problem definition.
   * @return A string representing the generated plan in PDDL format.
   */
  std::string generate_plan(const std::string domain_path,
                            const std::string problem_path) const override;

  using Planner::generate_plan;

  /**
   * @brief Checks if the generated plan indicates a valid solution.
   * @details Analyzes the plan output and determines if it represents a valid
   * solution to the planning problem.
   * @param plan_str The generated plan as a string.
   * @return True if the plan represents a valid solution, false otherwise.
   */
  bool has_solution(const std::string &plan_str) const override;

private:
  /// @brief Action cost value.
  float action_cost_;
  /// @brief Domain constraints parameter (0: prune static preconditions,
  /// otherwise keep).
  int domain_constraints_;
  /// @brief Flaw selection order.
  std::string flaw_order_;
  /// @brief Use ground actions.
  bool ground_actions_;
  /// @brief Heuristic to rank plans.
  std::string heuristic_;
  /// @brief Search no more than this number of plans.
  int limit_;
  /// @brief Add open conditions in random order.
  bool random_open_conditions_;
  /// @brief Seed for random number generator.
  int seed_;
  /// @brief Search algorithm.
  std::string search_algorithm_;
  /// @brief Time limit in minutes.
  int time_limit_;
  /// @brief Tolerance for durative actions.
  float tolerance_;
  /// @brief Weight for heuristic.
  float weight_;
};

} // namespace omni_plan_vhpop
#endif // OMNI_PLAN_VHPOP__VHPOP_PLANNER_HPP_
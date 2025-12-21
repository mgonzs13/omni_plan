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

#ifndef EASY_PLAN_SMTP__SMTP_PLANNER_HPP_
#define EASY_PLAN_SMTP__SMTP_PLANNER_HPP_

#include <memory>
#include <string>

#include "easy_plan/pddl/plan.hpp"
#include "easy_plan/planner.hpp"

namespace easy_plan_smtp {

/**
 * @class SmtpPlanner
 * @brief Planner implementation using the SMTP (Simple Temporal Problem)
 * algorithm.
 * @details This class provides automated planning capabilities using the SMTP
 * algorithm, which is particularly effective for temporal planning domains with
 * durative actions. SMTP can handle complex planning problems involving
 * concurrent actions and temporal constraints.
 */
class SmtpPlanner : public easy_plan::Planner {
public:
  /**
   * @brief Default constructor for SmtpPlanner.
   * @details Initializes the SMTP planner with default settings and prepares
   * it for solving planning problems.
   */
  SmtpPlanner();

  /**
   * @brief Generates a plan from PDDL domain and problem file paths.
   * @details Reads PDDL domain and problem definitions from files, then
   * generates a plan to solve the problem using the SMTP algorithm.
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
  /// @brief Begin iterative deepening at this number of happenings.
  int happenings_start_;
  /// @brief Run iterative deepening until this limit (-1 for unlimited).
  int happenings_limit_;
  /// @brief Limit the length of the concurrent cascading event and action
  /// chain.
  int chain_length_limit_;
  /// @brief Choose encoding (0: happening-based).
  int encoding_;
  /// @brief Iteratively deepen with this step size.
  int step_size_;
};

} // namespace easy_plan_smtp
#endif // EASY_PLAN_SMTP__SMTP_PLANNER_HPP_
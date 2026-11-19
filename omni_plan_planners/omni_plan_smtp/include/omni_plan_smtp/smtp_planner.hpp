// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OMNI_PLAN_SMTP__SMTP_PLANNER_HPP_
#define OMNI_PLAN_SMTP__SMTP_PLANNER_HPP_

#include <memory>
#include <string>

#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/planner.hpp"

namespace omni_plan_smtp {

/**
 * @class SmtpPlanner
 * @brief Planner implementation using the SMTP (Simple Temporal Problem)
 * algorithm.
 * @details This class provides automated planning capabilities using the SMTP
 * algorithm, which is particularly effective for temporal planning domains with
 * durative actions. SMTP can handle complex planning problems involving
 * concurrent actions and temporal constraints.
 */
class SmtpPlanner : public omni_plan::Planner {
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
  std::string generate_plan(const std::string &domain_path,
                            const std::string &problem_path) const override;

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
  /// @brief Maximum runtime in seconds (0 disables the timeout).
  int timeout_;
};

} // namespace omni_plan_smtp
#endif // OMNI_PLAN_SMTP__SMTP_PLANNER_HPP_
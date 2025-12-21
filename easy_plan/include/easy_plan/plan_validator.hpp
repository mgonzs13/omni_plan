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

#ifndef EASY_PLAN__PLAN_VALIDATOR_HPP_
#define EASY_PLAN__PLAN_VALIDATOR_HPP_

#include <string>

#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/plan.hpp"
#include "easy_plan/pddl/problem.hpp"

namespace easy_plan {

/**
 * @class PlanValidator
 * @brief Base class for validating planning solutions.
 * @details This abstract base class defines the interface for components that
 * validate whether a generated plan is correct and executable. Plan validators
 * check that the sequence of actions in a plan satisfies all preconditions and
 * achieves the goals.
 */
class PlanValidator {
public:
  /**
   * @brief Default constructor.
   */
  PlanValidator() = default;

  /**
   * @brief Default destructor.
   */
  virtual ~PlanValidator() = default;

  /**
   * @brief Validates a plan against a PDDL domain and problem.
   * @details Checks whether the given plan correctly solves the planning
   * problem defined by the domain and problem objects. This includes verifying
   * that all action preconditions are satisfied and that the goals are
   * achieved.
   * @param domain The PDDL domain definition.
   * @param problem The PDDL problem definition.
   * @param plan The plan to validate.
   * @return True if the plan is valid, false otherwise.
   */
  bool validate_plan(const pddl::Domain &domain, const pddl::Problem &problem,
                     pddl::Plan plan) const;

protected:
  /**
   * @brief Validates a plan against a PDDL domain and problem.
   * @details Checks whether the given plan correctly solves the planning
   * problem defined by the domain and problem strings. This includes verifying
   * that all action preconditions are satisfied and that the goals are
   * achieved.
   * @param domain The PDDL domain definition as a string.
   * @param problem The PDDL problem definition as a string.
   * @param plan The plan to validate.
   * @return True if the plan is valid, false otherwise.
   */
  virtual bool validate_plan(const std::string &domain_path,
                             const std::string &problem_path,
                             const std::string &plan_path) const = 0;

  /**
   * @brief Parses a PDDL plan string into a Plan object.
   * @details Converts a PDDL plan represented as a string into a Plan object
   * that can be processed and validated.
   * @param plan The PDDL plan as a string.
   * @return A Plan object representing the parsed plan.
   */
  virtual std::string parse_pddl(const pddl::Plan &plan) const = 0;
};

} // namespace easy_plan
#endif // EASY_PLAN__PLAN_VALIDATOR_HPP_
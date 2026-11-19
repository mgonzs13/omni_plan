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

#ifndef OMNI_PLAN__PLAN_VALIDATOR_HPP_
#define OMNI_PLAN__PLAN_VALIDATOR_HPP_

#include <string>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/utils/parameter_loader.hpp"

namespace omni_plan {

/**
 * @class PlanValidator
 * @brief Base class for validating planning solutions.
 * @details This abstract base class defines the interface for components that
 * validate whether a generated plan is correct and executable. Plan validators
 * check that the sequence of actions in a plan satisfies all preconditions and
 * achieves the goals.
 */
class PlanValidator : public utils::ParameterLoader {
public:
  /**
   * @brief Default constructor.
   */
  PlanValidator();

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
  virtual bool validate_plan(const pddl::Domain &domain,
                             const pddl::Problem &problem,
                             const pddl::Plan &plan) const;

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
};

} // namespace omni_plan
#endif // OMNI_PLAN__PLAN_VALIDATOR_HPP_
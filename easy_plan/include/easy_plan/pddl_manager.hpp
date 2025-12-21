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

#ifndef EASY_PLAN__PDDL_MANAGER_HPP_
#define EASY_PLAN__PDDL_MANAGER_HPP_

#include <set>
#include <string>

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan/utils/parameter_loader.hpp"

namespace easy_plan {

/**
 * @class PddlManager
 * @brief Base class for managing PDDL domain and problem generation.
 * @details This abstract base class defines the interface for components that
 * generate PDDL domains and problems from action definitions and current state.
 * It provides methods for checking goals, validating predicates, and applying
 * effects to maintain the current state representation.
 */
class PddlManager : public utils::ParameterLoader {
public:
  /**
   * @brief Default constructor.
   */
  PddlManager();

  /**
   * @brief Default destructor.
   */
  virtual ~PddlManager() = default;

  /**
   * @brief Generates PDDL domain and problem from a list of actions.
   * @details Creates a complete PDDL domain and problem pair based on the
   * provided actions. This method extracts types, predicates, and other domain
   * elements from the action definitions.
   * @param actions A vector of shared pointers to Action objects.
   * @return A pair containing the generated Domain and Problem objects.
   */
  std::pair<pddl::Domain, pddl::Problem>
  get_pddl(std::vector<std::shared_ptr<pddl::Action>> actions) const;

  /**
   * @brief Pure virtual method to generate PDDL domain and problem.
   * @details Derived classes must implement this method to provide the specific
   * logic for generating PDDL based on their internal state representation.
   * @return A pair containing the generated Domain and Problem objects.
   */
  virtual std::pair<pddl::Domain, pddl::Problem> get_pddl() const = 0;

  /**
   * @brief Checks if there are any goals to achieve.
   * @details Determines whether the current planning problem has any goals
   * that need to be satisfied.
   * @return True if goals exist, false otherwise.
   */
  virtual bool has_goals() const = 0;

  /**
   * @brief Checks if a predicate exists in the current state.
   * @details Verifies whether the given predicate is currently true in the
   * world state.
   * @param predicate The predicate to check.
   * @return True if the predicate exists, false otherwise.
   */
  virtual bool predicate_exists(const pddl::Predicate &predicate) const = 0;

  /**
   * @brief Checks if a predicate is part of the goal conditions.
   * @details Determines whether the given predicate needs to be achieved as
   * part of the goals.
   * @param predicate The predicate to check.
   * @return True if the predicate is a goal, false otherwise.
   */
  virtual bool
  predicate_is_goal(const easy_plan::pddl::Predicate &predicate) const = 0;

  /**
   * @brief Applies a single effect to the current state.
   * @details Updates the world state by applying the specified effect, which
   * may add or delete predicates from the current state representation.
   * @param exp The effect to apply.
   */
  virtual void apply_effect(const pddl::Effect &exp) = 0;

  /**
   * @brief Applies multiple effects to the current state.
   * @details Updates the world state by applying all the specified effects in
   * sequence.
   * @param effects A vector of effects to apply.
   * @return A vector of the applied effects (for potential rollback or
   * logging).
   */
  std::vector<pddl::Effect>
  apply_effects(const std::vector<pddl::Effect> &effects);

private:
  /**
   * @brief Extracts the types used by an action's parameters.
   * @details Analyzes an action's parameters to determine all the object types
   * that need to be defined in the PDDL domain.
   * @param action A shared pointer to the action to analyze.
   * @return A set of type names used by the action.
   */
  std::set<std::string>
  get_actions_types(std::shared_ptr<easy_plan::pddl::Action> action) const;

  /**
   * @brief Converts a predicate to use action parameter names.
   * @details Transforms predicate arguments from generic names to the specific
   * parameter names defined in the action.
   * @param pred The predicate to convert.
   * @param action The action containing the parameter definitions.
   * @return The converted predicate with action parameter names.
   */
  easy_plan::pddl::Predicate convert_action_predicate(
      easy_plan::pddl::Predicate pred,
      std::shared_ptr<easy_plan::pddl::Action> action) const;

  /**
   * @brief Extracts all predicates used by an action.
   * @details Collects all predicates that appear in the action's conditions and
   * effects.
   * @param action The action to analyze.
   * @return A set of all predicates used by the action.
   */
  std::set<easy_plan::pddl::Predicate>
  get_action_predicates(std::shared_ptr<easy_plan::pddl::Action> action) const;

  /// @brief Requirements for the PDDL domain.
  std::vector<std::string> domain_requirements;
};

} // namespace easy_plan
#endif // EASY_PLAN__PDDL_MANAGER_HPP_
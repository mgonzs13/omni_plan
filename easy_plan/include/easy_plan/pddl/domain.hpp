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

#ifndef EASY_PLAN__DOMAIN_HPP_
#define EASY_PLAN__DOMAIN_HPP_

#include <memory>
#include <set>
#include <string>

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/predicate.hpp"

namespace easy_plan {
namespace pddl {

/**
 * @class Domain
 * @brief Represents a PDDL domain with its requirements, types, predicates, and
 * actions.
 * @details A PDDL domain defines the structure and rules of a planning problem,
 * including the types of objects, predicates for describing states, and actions
 * that can change the state. This class provides methods to build and
 * manipulate domain components and generate PDDL output.
 */
class Domain {
public:
  /**
   * @brief Constructs an empty Domain.
   */
  Domain() = default;

  /**
   * @brief Adds a requirement to the domain.
   * @details Requirements specify the PDDL features used in the domain,
   * such as :strips, :typing, :durative-actions, etc.
   * @param requirement The requirement to add (e.g., ":strips").
   */
  void add_requirement(const std::string &requirement);

  /**
   * @brief Adds a type to the domain.
   * @details Types define the categories of objects in the domain,
   * enabling type checking and hierarchical organization.
   * @param type The type to add.
   */
  void add_type(const std::string &type);

  /**
   * @brief Adds a predicate to the domain.
   * @details Predicates define the state variables that can be true or false
   * in the planning problem.
   * @param pred The predicate to add.
   */
  void add_predicate(const Predicate &pred);

  /**
   * @brief Adds an action to the domain.
   * @details Actions define the operators that can change the state of the
   * world.
   * @param action The action to add.
   */
  void add_action(const std::shared_ptr<Action> &action);

  /**
   * @brief Converts the domain to its PDDL representation.
   * @details Generates a complete PDDL domain file content as a string,
   * including all requirements, types, predicates, and actions.
   * @return A string representing the domain in PDDL format.
   */
  std::string to_pddl() const;

private:
  /// The requirements of the domain.
  std::set<std::string> requirements_;
  /// The types defined in the domain.
  std::set<std::string> types_;
  /// The predicates defined in the domain.
  std::set<Predicate> predicates_;
  /// The actions defined in the domain.
  std::set<std::shared_ptr<Action>> actions_;
};

} // namespace pddl
} // namespace easy_plan

#endif // EASY_PLAN__DOMAIN_HPP_

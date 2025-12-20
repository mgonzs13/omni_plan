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
#include "easy_plan/pddl/expression.hpp"

namespace easy_plan {

/**
 * @brief Base class for PDDL managers.
 */
class PddlManager {
public:
  /**
   * @brief Default constructor
   */
  PddlManager() = default;

  /**
   * @brief Default destructor
   */
  virtual ~PddlManager() = default;

  std::pair<std::string, std::string> get_pddl() const;

  virtual std::pair<std::string, std::string>
  get_pddl(std::set<std::string> actions_types,
           std::set<std::string> actions_predicates,
           std::set<std::string> actions_pddl) const = 0;

  virtual bool has_goals() const = 0;

  virtual bool predicate_exists(const pddl::Predicate &predicate) const = 0;

  virtual bool
  predicate_is_goal(const easy_plan::pddl::Predicate &predicate) const = 0;

  virtual void apply_effect(const pddl::Effect &exp) = 0;

  std::vector<pddl::Effect>
  apply_effects(const std::vector<pddl::Effect> &effects);
};

} // namespace easy_plan
#endif // EASY_PLAN__PDDL_MANAGER_HPP_
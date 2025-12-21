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

#ifndef EASY_PLAN__PLAN_HPP_
#define EASY_PLAN__PLAN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "easy_plan/pddl/action.hpp"

namespace easy_plan {
namespace pddl {

/**
 * @class Plan
 * @brief Represents a sequence of actions that solves a planning problem.
 * @details This class stores a sequence of actions with their parameters that,
 * when executed in order, achieve the goals of a planning problem. It also
 * tracks whether a valid solution was found.
 */
class Plan {
public:
  /**
   * @brief Constructs a Plan with an optional solution status.
   * @param has_solution Whether this plan represents a valid solution (default:
   * false).
   */
  Plan(bool has_solution = false);

  /**
   * @brief Checks if this plan represents a valid solution.
   * @return True if the plan has a solution, false otherwise.
   */
  bool has_solution() const;

  /**
   * @brief Adds an action with its parameters to the plan.
   * @details Appends an action and its execution parameters to the end of the
   * plan.
   * @param action A shared pointer to the action to add.
   * @param params The parameters for executing the action (default: empty
   * vector).
   */
  void add_action(const std::shared_ptr<pddl::Action> &action,
                  const std::vector<std::string> &params = {});

  /**
   * @brief Gets the number of actions in the plan.
   * @return The size of the plan (number of actions).
   */
  size_t size() const;

  /**
   * @brief Gets the action at a specific index in the plan.
   * @param index The index of the action to retrieve.
   * @return A shared pointer to the action at the specified index.
   */
  std::shared_ptr<pddl::Action> get_action(size_t index) const;

  /**
   * @brief Gets the parameters for the action at a specific index.
   * @param index The index of the action whose parameters to retrieve.
   * @return A vector of parameter strings for the action.
   */
  std::vector<std::string> get_action_params(size_t index) const;

  /**
   * @brief Gets both the action and its parameters at a specific index.
   * @param index The index of the action to retrieve.
   * @return A pair containing the action and its parameters.
   */
  std::pair<std::shared_ptr<pddl::Action>, std::vector<std::string>>
  get_action_with_params(size_t index) const;

private:
  /// Whether this plan represents a valid solution.
  bool has_solution_ = false;
  /// The sequence of actions in the plan.
  std::vector<std::shared_ptr<pddl::Action>> actions_;
  /// The parameters for each action in the plan.
  std::vector<std::vector<std::string>> params_;
  /// Mutable index for internal iteration (used by some methods).
  mutable size_t current_index_ = 0;
};

} // namespace pddl
} // namespace easy_plan
#endif // EASY_PLAN__PLAN_HPP_
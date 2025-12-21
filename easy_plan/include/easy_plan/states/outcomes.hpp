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

#ifndef EASY_PLAN__STATES__OUTCOMES_HPP_
#define EASY_PLAN__STATES__OUTCOMES_HPP_

/**
 * @file outcomes.hpp
 * @brief Defines string constants for state machine outcomes in the easy_plan
 * framework.
 * @details This header provides standardized outcome strings used by state
 * machines to communicate the results of various planning operations, such as
 * goal checking and plan validation.
 */

namespace easy_plan {
namespace states {

/**
 * @namespace outcomes
 * @brief Contains string constants representing possible outcomes of planning
 * operations.
 * @details These constants are used as return values or status indicators in
 * state machines to signal the success or failure of operations like goal
 * evaluation and plan validation.
 */
namespace outcomes {

/**
 * @brief Outcome indicating that goals are present and need to be achieved.
 */
static const char *const HAS_GOALS = "has_goals";

/**
 * @brief Outcome indicating that no goals are currently defined.
 */
static const char *const NO_GOALS = "no_goals";

/**
 * @brief Outcome indicating that a validation or check operation was
 * successful.
 */
static const char *const VALID = "valid";

/**
 * @brief Outcome indicating that a validation or check operation failed.
 */
static const char *const INVALID = "invalid";

} // namespace outcomes
} // namespace states
} // namespace easy_plan

#endif // EASY_PLAN__STATES__OUTCOMES_HPP_
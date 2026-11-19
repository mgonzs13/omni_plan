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

#ifndef OMNI_PLAN__STATES__OUTCOMES_HPP_
#define OMNI_PLAN__STATES__OUTCOMES_HPP_

/**
 * @file outcomes.hpp
 * @brief Defines string constants for state machine outcomes in the omni_plan
 * framework.
 * @details This header provides standardized outcome strings used by state
 * machines to communicate the results of various planning operations, such as
 * goal checking and plan validation.
 */

namespace omni_plan {
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
} // namespace omni_plan

#endif // OMNI_PLAN__STATES__OUTCOMES_HPP_
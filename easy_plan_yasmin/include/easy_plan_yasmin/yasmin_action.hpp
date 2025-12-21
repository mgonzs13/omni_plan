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

#ifndef EASY_PLAN_YASMIN__YASMIN_ACTION_HPP_
#define EASY_PLAN_YASMIN__YASMIN_ACTION_HPP_

#include <memory>
#include <string>

#include "yasmin/state_machine.hpp"

#include "easy_plan/pddl/action.hpp"

namespace easy_plan_yasmin {

/**
 * @brief YasminAction class that integrates Yasmin state machines with Easy
 * Plan actions. Inherits from both easy_plan::pddl::Action and
 * yasmin::StateMachine. This class allows the execution of Yasmin state
 * machines as actions within the Easy Plan framework.
 */
class YasminAction : public easy_plan::pddl::Action,
                     public yasmin::StateMachine {
public:
  /**
   * @brief Constructs a YasminAction with the given name, parameters, and an
   * optional state machine.
   * @param name The name of the action.
   * @param params The parameters of the action (default is an empty vector).
   */
  YasminAction(
      const std::string &name,
      const std::vector<std::pair<std::string, std::string>> &params = {});

  /**
   * @brief Virtual destructor for the YasminAction class.
   */
  ~YasminAction() override = default;

  /**
   * @brief Executes the Yasmin state machine as an action.
   * @param params The parameters for the action execution.
   * @return The status of the action execution (SUCCEED, CANCEL, ABORT).
   */
  easy_plan::pddl::ActionStatus
  run(const std::vector<std::string> &params) override;

  /**
   * @brief Cancels the execution of the Yasmin state machine.
   */
  void cancel() override;
};

} // namespace easy_plan_yasmin
#endif // EASY_PLAN_YASMIN__YASMIN_ACTION_HPP_
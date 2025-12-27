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

#ifndef EASY_PLAN_YASMIN__YASMIN_FACTORY_ACTION_HPP_
#define EASY_PLAN_YASMIN__YASMIN_FACTORY_ACTION_HPP_

#include <memory>
#include <string>

#include "yasmin/state_machine.hpp"
#include "yasmin_factory/yasmin_factory.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

#include "easy_plan/pddl/action.hpp"

namespace easy_plan_yasmin {

/**
 * @brief YasminFactoryAction class that integrates a Yasmin state machine
 * with Easy Plan's Action interface.
 */
class YasminFactoryAction : public easy_plan::pddl::Action {
public:
  /**
   * @brief Constructs a YasminFactoryAction with a given name, parameters, and
   * state machine XML definition.
   * @param name The name of the action.
   * @param params The parameters of the action (default is an empty vector).
   * @param state_machine_xml The XML definition of the Yasmin state machine
   * (default is an empty string).
   */
  YasminFactoryAction(
      const std::string &name,
      const std::vector<std::pair<std::string, std::string>> &params = {});

  /**
   * @brief Virtual destructor for the YasminFactoryAction class.
   */
  ~YasminFactoryAction() override = default;

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

private:
  /// @brief Flag to enable the Yasmin Viewer publisher.
  bool enable_viewer_pub_;
  /// @brief Yasmin Viewer publisher for visualizing the state machine
  /// execution.
  std::unique_ptr<yasmin_viewer::YasminViewerPub> viewer_pub_;
  /// @brief Yasmin Factory for creating state machines from XML.
  std::unique_ptr<yasmin_factory::YasminFactory> factory_;
  /// @brief XML definition of the Yasmin state machine.
  std::string state_machine_xml_;
  /// @brief Shared pointer to the Yasmin state machine instance.
  std::shared_ptr<yasmin::StateMachine> state_machine_;
};

} // namespace easy_plan_yasmin
#endif // EASY_PLAN_YASMIN__YASMIN_FACTORY_ACTION_HPP_
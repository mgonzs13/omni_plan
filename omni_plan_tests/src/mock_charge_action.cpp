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

#include <atomic>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"

namespace omni_plan_tests {

/**
 * @brief MockChargeAction - A test action plugin for robot charging
 *
 * This action simulates robot charging at a charging station. It's designed for
 * integration testing of the omni_plan system with the YASMIN state machine.
 */
class MockChargeAction : public omni_plan::pddl::Action {
public:
  MockChargeAction()
      : Action("charge", {{"robot", "robot"}, {"room", "room"}}) {
    // Conditions
    add_condition(omni_plan::pddl::Type::START, "robot_at", {"robot", "room"});
    add_condition(omni_plan::pddl::Type::START, "charging_point_at", {"room"});
    add_condition(omni_plan::pddl::Type::START, "battery_low", {"robot"});

    // Effects
    add_effect(omni_plan::pddl::Type::END, "battery_low", {"robot"}, true);
    add_effect(omni_plan::pddl::Type::END, "battery_full", {"robot"});
  }

  omni_plan::pddl::ActionStatus
  run(const std::vector<std::string> &params) override {
    execution_count_++;
    last_params_ = params;
    return omni_plan::pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override { cancel_called_ = true; }

  // Static accessors for test verification
  static int get_execution_count() { return execution_count_; }
  static bool was_cancel_called() { return cancel_called_; }
  static std::vector<std::string> get_last_params() { return last_params_; }
  static void reset() {
    execution_count_ = 0;
    cancel_called_ = false;
    last_params_.clear();
  }

private:
  static std::atomic<int> execution_count_;
  static std::atomic<bool> cancel_called_;
  static std::vector<std::string> last_params_;
};

std::atomic<int> MockChargeAction::execution_count_{0};
std::atomic<bool> MockChargeAction::cancel_called_{false};
std::vector<std::string> MockChargeAction::last_params_;

} // namespace omni_plan_tests

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(omni_plan_tests::MockChargeAction,
                       omni_plan::pddl::Action)

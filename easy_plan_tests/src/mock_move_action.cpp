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

#include <atomic>
#include <string>
#include <vector>

#include "easy_plan/pddl/action.hpp"

namespace easy_plan_tests {

/**
 * @brief MockMoveAction - A test action plugin for robot movement
 *
 * This action simulates robot movement between rooms. It's designed for
 * integration testing of the easy_plan system with the YASMIN state machine.
 */
class MockMoveAction : public easy_plan::pddl::Action {
public:
  MockMoveAction()
      : Action("move", {{"robot", "robot"}, {"r1", "room"}, {"r2", "room"}}) {
    // Conditions
    add_condition(easy_plan::pddl::Type::START, "robot_at", {"robot", "r1"});
    add_condition(easy_plan::pddl::Type::START, "connected", {"r1", "r2"});

    // Effects
    add_effect(easy_plan::pddl::Type::START, "robot_at", {"robot", "r1"}, true);
    add_effect(easy_plan::pddl::Type::END, "robot_at", {"robot", "r2"});
  }

  easy_plan::pddl::ActionStatus
  run(const std::vector<std::string> &params) override {
    execution_count_++;
    last_params_ = params;
    return easy_plan::pddl::ActionStatus::SUCCEED;
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

std::atomic<int> MockMoveAction::execution_count_{0};
std::atomic<bool> MockMoveAction::cancel_called_{false};
std::vector<std::string> MockMoveAction::last_params_;

} // namespace easy_plan_tests

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easy_plan_tests::MockMoveAction, easy_plan::pddl::Action)

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

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include "easy_plan/pddl/action.hpp"

using namespace easy_plan;

class ChargeAction : public pddl::Action {
public:
  ChargeAction()
      : Action("charge",
               {
                   {"robot", "robot"},
                   {"room", "room"},
               }),
        progress_(0.0) {

    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "room"});
    this->add_condition(pddl::START, "charging_point_at",
                        std::vector<std::string>{"room"});

    this->add_effect(pddl::END, "battery_low",
                     std::vector<std::string>{"robot"}, true);
    this->add_effect(pddl::END, "battery_full",
                     std::vector<std::string>{"robot"});

    this->add_ros_parameters({
        {"increment", 0.05f, this->increment_},
    });
  }

  pddl::ActionStatus run(const std::vector<std::string> &params) override {
    std::string robot = params[0];
    std::cout << robot << " charging " << std::endl;

    while (this->progress_ < 1.0) {
      this->progress_ += this->increment_;
      std::cout << "Charging ... [" << std::min(100.0, this->progress_ * 100.0)
                << "%]  " << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    this->progress_ = 0.0;
    std::cout << std::endl;
    return pddl::ActionStatus::SUCCEED;
  }

  void cancel() override {
    // Handle cancellation if needed
    std::cout << "Ask charge action cancelled." << std::endl;
  }

private:
  /// @brief Progress of the action (0.0 to 1.0).
  float progress_ = 0.0;
  /// @brief Increment per iteration.
  float increment_ = 0.05;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ChargeAction, easy_plan::pddl::Action)
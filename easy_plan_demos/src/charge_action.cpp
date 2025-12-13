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

#include "easy_plan/pddl/action.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

using namespace easy_plan;

class ChargeAction : public pddl::Action {
public:
  ChargeAction()
      : Action("charge", {
                             pddl::Parameter("robot", "robot"),
                             pddl::Parameter("room", "room"),

                         }) {
    this->add_condition(
        pddl::Condition::START,
        std::make_shared<pddl::Predicate>(
            "robot_at", std::vector<std::string>{"robot", "room"}));
    this->add_condition(
        pddl::Condition::START,
        std::make_shared<pddl::Predicate>("charging_point_at",
                                          std::vector<std::string>{"room"}));
    this->add_effect(
        pddl::Effect::END,
        std::make_shared<pddl::Predicate>(
            "battery_low", std::vector<std::string>{"robot"}, true));
    this->add_effect(pddl::Effect::END,
                     std::make_shared<pddl::Predicate>(
                         "battery_full", std::vector<std::string>{"robot"}));
  }

  pddl::ActionStatus run(std::vector<std::string> params) override {
    std::string robot = params[0];
    std::cout << robot << " charging " << std::endl;

    while (this->progress_ < 1.0) {
      this->progress_ += 0.05;
      std::cout << "\r\033[K" << std::flush;
      std::cout << "Charging ... [" << std::min(100.0, this->progress_ * 100.0)
                << "%]  " << std::flush;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    this->progress_ = 0.0;
    std::cout << std::endl;
    return pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override {
    // Handle cancellation if needed
    std::cout << "Ask charge action cancelled." << std::endl;
  }

private:
  float progress_ = 0.0;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ChargeAction, easy_plan::pddl::Action)
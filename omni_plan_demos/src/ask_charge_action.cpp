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

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include "omni_plan/pddl/action.hpp"

using namespace omni_plan;

class AskChargeAction : public pddl::Action {
public:
  AskChargeAction()
      : Action("ask_charge",
               {
                   {"robot", "robot"},
                   {"r1", "room"},
                   {"r2", "room"},
               }),
        progress_(0.0) {

    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "r1"});
    this->add_condition(pddl::START, "charging_point_at",
                        std::vector<std::string>{"r2"});

    this->add_effect(pddl::END, "robot_at",
                     std::vector<std::string>{"robot", "r1"}, true);
    this->add_effect(pddl::END, "robot_at",
                     std::vector<std::string>{"robot", "r2"});

    this->add_ros_parameters({
        {"increment", 0.05f, this->increment_},
    });
  }

  pddl::ActionStatus run(const std::vector<std::string> &params) override {
    std::string robot = params[0];
    std::cout << "Asking " << robot << " to charge." << std::endl;

    while (this->progress_ < 1.0) {
      this->progress_ += this->increment_;
      std::cout << "Requesting for charging ... ["
                << std::min(100.0, this->progress_ * 100.0) << "%]  "
                << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
  /// @brief Progress of the action (0.0 to 1.0).
  float progress_ = 0.0;
  /// @brief Increment per iteration.
  float increment_ = 0.05;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AskChargeAction, omni_plan::pddl::Action)
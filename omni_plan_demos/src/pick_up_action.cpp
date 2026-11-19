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

/**
 * @brief Action that picks up a component from a room.
 */
class PickUpAction : public pddl::Action {
public:
  PickUpAction()
      : Action("pick_up", {
                              {"robot", "robot"},
                              {"comp", "component"},
                              {"room", "room"},
                          }) {

    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "room"});
    this->add_condition(pddl::START, "component_at",
                        std::vector<std::string>{"comp", "room"});

    this->add_effect(pddl::END, "carrying",
                     std::vector<std::string>{"robot", "comp"});
    this->add_effect(pddl::END, "component_at",
                     std::vector<std::string>{"comp", "room"}, true);

    this->add_ros_parameters({
        {"increment", 0.05f, this->increment_},
    });
  }

  pddl::ActionStatus run(const std::vector<std::string> &params) override {
    std::string robot = params[0];
    std::string comp = params[1];
    std::string room = params[2];

    std::cout << robot << " picking up " << comp << " at " << room << std::endl;

    while (this->progress_ < 1.0) {
      this->progress_ += this->increment_;
      std::cout << "[" << robot << "] Picking up " << comp << " ... ["
                << std::min(100.0, this->progress_ * 100.0) << "%]"
                << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    this->progress_ = 0.0;
    std::cout << robot << " picked up " << comp << std::endl;
    return pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override {
    std::cout << "PickUp action cancelled." << std::endl;
  }

private:
  float progress_ = 0.0;
  float increment_ = 0.1;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PickUpAction, omni_plan::pddl::Action)

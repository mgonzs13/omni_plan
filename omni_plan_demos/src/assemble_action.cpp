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
 * @brief Action that assembles two components brought by two robots.
 */
class AssembleAction : public pddl::Action {
public:
  AssembleAction()
      : Action("assemble", {
                               {"assembler", "robot"},
                               {"helper", "robot"},
                               {"comp1", "component"},
                               {"comp2", "component"},
                               {"room", "room"},
                           }) {

    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"assembler", "room"});
    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"helper", "room"});
    this->add_condition(pddl::START, "is_assembly_room",
                        std::vector<std::string>{"room"});
    this->add_condition(pddl::START, "carrying",
                        std::vector<std::string>{"assembler", "comp1"});
    this->add_condition(pddl::START, "carrying",
                        std::vector<std::string>{"helper", "comp2"});

    this->add_effect(pddl::END, "assembled",
                     std::vector<std::string>{"comp1", "comp2"});

    this->add_ros_parameters({
        {"increment", 0.05f, this->increment_},
    });
  }

  pddl::ActionStatus run(const std::vector<std::string> &params) override {
    std::string assembler = params[0];
    std::string helper = params[1];
    std::string comp1 = params[2];
    std::string comp2 = params[3];
    std::string room = params[4];

    std::cout << assembler << " assembling " << comp1 << " and " << comp2
              << " (helped by " << helper << ") in " << room << std::endl;

    while (this->progress_ < 1.0) {
      this->progress_ += this->increment_;
      std::cout << "[" << assembler << "] Assembling " << comp1 << " + "
                << comp2 << " ... [" << std::min(100.0, this->progress_ * 100.0)
                << "%]" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    this->progress_ = 0.0;
    std::cout << assembler << " successfully assembled " << comp1 << " and "
              << comp2 << "!" << std::endl;
    return pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override {
    std::cout << "Assemble action cancelled." << std::endl;
  }

private:
  float progress_ = 0.0;
  float increment_ = 0.05;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AssembleAction, omni_plan::pddl::Action)

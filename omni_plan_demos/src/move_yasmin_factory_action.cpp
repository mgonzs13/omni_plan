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

#include "omni_plan/utils/package_share_path.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan_yasmin/yasmin_factory_action.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

using namespace omni_plan;

/**
 * @brief MoveFactoryAction class that uses YasminAction to execute
 * a move action using a state machine defined in C++.
 */
class MoveYasminFactoryAction : public omni_plan_yasmin::YasminFactoryAction {
public:
  MoveYasminFactoryAction()
      : YasminFactoryAction(
            "move",
            {
                {"robot", "robot"},
                {"r1", "room"},
                {"r2", "room"},
            },
            omni_plan::utils::get_package_share_path("omni_plan_demos") +
                "/state_machines/move_action_sm.xml") {

    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "r1"});
    this->add_condition(pddl::OVER_ALL, "battery_full",
                        std::vector<std::string>{"robot"});
    this->add_condition(pddl::START, "connected",
                        std::vector<std::string>{"r1", "r2"});

    this->add_effect(pddl::START, "robot_at",
                     std::vector<std::string>{"robot", "r1"}, true);
    this->add_effect(pddl::END, "robot_at",
                     std::vector<std::string>{"robot", "r2"});

    this->add_ros_parameters({
        {"increment", 0.05f, this->increment_},
    });
  }

  yasmin::Blackboard::SharedPtr create_blackboard() override {
    auto blackboard = std::make_shared<yasmin::Blackboard>();
    blackboard->set<float>("increment", this->increment_);
    return blackboard;
  }

private:
  /// @brief Increment per iteration.
  float increment_ = 0.05;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MoveYasminFactoryAction, omni_plan::pddl::Action)

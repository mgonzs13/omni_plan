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

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "easy_plan/pddl/action.hpp"
#include "easy_plan_yasmin/yasmin_factory_action.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

using namespace easy_plan;

/**
 * @brief MoveFactoryAction class that uses YasminAction to execute
 * a move action using a state machine defined in C++.
 */
class MoveYasminFactoryAction : public easy_plan_yasmin::YasminFactoryAction {
public:
  MoveYasminFactoryAction()
      : YasminFactoryAction(
            "move",
            {
                {"robot", "robot"},
                {"r1", "room"},
                {"r2", "room"},
            },
            ament_index_cpp::get_package_share_directory("easy_plan_demos") +
                "/state_machines/move_action_sm.xml") {

    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "r1"});
    this->add_condition(pddl::OVER_ALL, "battery_full",
                        std::vector<std::string>{"robot"});
    this->add_condition(pddl::START, "connected",
                        std::vector<std::string>{"r1", "r2"});

    this->add_effect(pddl::END, "robot_at",
                     std::vector<std::string>{"robot", "r1"}, true);
    this->add_effect(pddl::END, "robot_at",
                     std::vector<std::string>{"robot", "r2"});
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MoveYasminFactoryAction, easy_plan::pddl::Action)

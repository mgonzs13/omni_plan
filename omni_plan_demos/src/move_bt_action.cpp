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

#include "omni_plan/pddl/action.hpp"
#include "omni_plan_bt/bt_action.hpp"

using namespace omni_plan;

/**
 * @brief MoveBtAction class that uses BtAction to execute
 * a move action using a behavior tree defined in XML.
 */
class MoveBtAction : public omni_plan_bt::BtAction {
public:
  MoveBtAction()
      : BtAction(
            "move",
            {
                {"robot", "robot"},
                {"r1", "room"},
                {"r2", "room"},
            },
            ament_index_cpp::get_package_share_directory("omni_plan_demos") +
                "/behavior_trees/move_action_bt.xml") {

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

    this->add_ros_parameters({
        {"increment", 0.05f, this->increment_},
    });
  }

  void load_data_in_blackboard() override {
    this->blackboard_->set("increment", this->increment_);
  }

private:
  /// @brief Increment per iteration.
  float increment_ = 0.05;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MoveBtAction, omni_plan::pddl::Action)

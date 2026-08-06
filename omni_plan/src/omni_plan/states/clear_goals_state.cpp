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

#include <memory>
#include <string>

#include "poirot/poirot.hpp"
#include "yasmin/state.hpp"

#include "omni_plan/pddl_manager.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

class ClearGoalsState : public yasmin::State {

public:
  ClearGoalsState()
      : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                       yasmin_ros::basic_outcomes::ABORT}) {
    this->set_description(
        "State responsible for clearing all goals from the PDDL manager.");
    this->set_outcome_description(yasmin_ros::basic_outcomes::SUCCEED,
                                  "All goals have been cleared.");
    this->set_outcome_description(yasmin_ros::basic_outcomes::ABORT,
                                  "Failed to clear goals.");
    this->add_input_key("pddl_manager", "The PDDL manager.");
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) {
    PROFILE_FUNCTION();

    try {
      auto pddl_manager =
          blackboard->get<std::shared_ptr<omni_plan::PddlManager>>(
              "pddl_manager");

      if (!pddl_manager) {
        return yasmin_ros::basic_outcomes::SUCCEED;
      }

      pddl_manager->clear_goals();

    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to clear goals: %s", e.what());
      return yasmin_ros::basic_outcomes::ABORT;
    }

    return yasmin_ros::basic_outcomes::SUCCEED;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ClearGoalsState, yasmin::State)
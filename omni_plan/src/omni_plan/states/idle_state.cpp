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
#include "yasmin_ros/basic_outcomes.hpp"

#include "omni_plan/pddl_manager.hpp"
#include "omni_plan/states/outcomes.hpp"

class IdleState : public yasmin::State {

public:
  IdleState()
      : yasmin::State({omni_plan::states::outcomes::HAS_GOALS,
                       omni_plan::states::outcomes::NO_GOALS,
                       yasmin_ros::basic_outcomes::ABORT}) {
    this->set_description(
        "Idle state. Checks if there are goals to be achieved.");
    this->set_outcome_description(omni_plan::states::outcomes::HAS_GOALS,
                                  "There are goals to be achieved.");
    this->set_outcome_description(omni_plan::states::outcomes::NO_GOALS,
                                  "There are no goals to be achieved.");
    this->set_outcome_description(yasmin_ros::basic_outcomes::ABORT,
                                  "Failed to check goals.");
    this->add_input_key("pddl_manager", "The PDDL manager.");
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) {
    PROFILE_FUNCTION();

    try {
      auto pddl_manager =
          blackboard->get<std::shared_ptr<omni_plan::PddlManager>>(
              "pddl_manager");

      if (pddl_manager && pddl_manager->has_goals()) {
        return omni_plan::states::outcomes::HAS_GOALS;
      }

    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to check goals: %s", e.what());
      return yasmin_ros::basic_outcomes::ABORT;
    }

    return omni_plan::states::outcomes::NO_GOALS;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(IdleState, yasmin::State)
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
#include <vector>

#include "poirot/poirot.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl_manager.hpp"

class GeneratePddlState : public yasmin::State {

public:
  GeneratePddlState()
      : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                       yasmin_ros::basic_outcomes::ABORT}) {
    this->set_description(
        "State responsible for generating the PDDL domain and problem from the "
        "actions on the blackboard. The generated domain and problem are also "
        "stored on the blackboard for later use by the planner.");
    this->set_outcome_description(
        yasmin_ros::basic_outcomes::SUCCEED,
        "PDDL domain and problem successfully generated");
    this->add_input_key("actions", "Map of action name to Action plugin, "
                                   "expected to be on the blackboard");
    this->add_input_key("pddl_manager",
                        "The PDDL manager, expected to be on the blackboard");
    this->add_output_key("domain", "The generated PDDL domain");
    this->add_output_key("problem", "The generated PDDL problem");
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) {
    PROFILE_FUNCTION();

    try {
      auto pddl_manager =
          blackboard->get<std::shared_ptr<omni_plan::PddlManager>>(
              "pddl_manager");
      auto actions_maps = blackboard->get<std::unordered_map<
          std::string, std::shared_ptr<omni_plan::pddl::Action>>>("actions");

      std::vector<std::shared_ptr<omni_plan::pddl::Action>> actions;
      for (const auto &action_pair : actions_maps) {
        actions.push_back(action_pair.second);
      }

      auto [domain, problem] = pddl_manager->get_pddl(actions);

      blackboard->set<omni_plan::pddl::Domain>("domain", domain);
      blackboard->set<omni_plan::pddl::Problem>("problem", problem);

      YASMIN_LOG_INFO("PDDL domain generated:\n%s", domain.to_pddl().c_str());
      YASMIN_LOG_INFO("PDDL problem generated:\n%s", problem.to_pddl().c_str());
      return yasmin_ros::basic_outcomes::SUCCEED;
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to generate PDDL: %s", e.what());
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GeneratePddlState, yasmin::State)
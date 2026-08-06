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

#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/planner.hpp"

class PlanState : public yasmin::State {

public:
  PlanState()
      : yasmin::State({
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::ABORT,
        }) {
    this->set_description("Generate a plan using the planner plugin.");
    this->set_outcome_description(yasmin_ros::basic_outcomes::SUCCEED,
                                  "Plan generated successfully.");
    this->set_outcome_description(yasmin_ros::basic_outcomes::ABORT,
                                  "Failed to generate a plan.");
    this->add_input_key("planner", "The planner plugin.");
    this->add_input_key("domain", "The PDDL domain.");
    this->add_input_key("problem", "The PDDL problem.");
    this->add_input_key("actions", "The available actions.");
    this->add_output_key("plan", "The generated plan.");
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) {
    PROFILE_FUNCTION();

    try {
      auto planner =
          blackboard->get<std::shared_ptr<omni_plan::Planner>>("planner");

      blackboard->set<omni_plan::pddl::Plan>(
          "plan", planner->generate_plan(
                      blackboard->get<omni_plan::pddl::Domain>("domain"),
                      blackboard->get<omni_plan::pddl::Problem>("problem")));
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Plan generation failed: %s", e.what());
      return yasmin_ros::basic_outcomes::ABORT;
    }

    YASMIN_LOG_INFO("Planner output: %s",
                    blackboard->get<omni_plan::pddl::Plan>("plan")
                        .get_raw_output()
                        .c_str());

    if (!blackboard->get<omni_plan::pddl::Plan>("plan").has_solution()) {
      YASMIN_LOG_WARN("Planner could not find a valid plan");
      return yasmin_ros::basic_outcomes::ABORT;
    }

    return yasmin_ros::basic_outcomes::SUCCEED;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PlanState, yasmin::State)
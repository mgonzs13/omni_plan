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

#include <memory>
#include <string>

#include "poirot/poirot.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/plan_validator.hpp"
#include "omni_plan/states/outcomes.hpp"

class ValidatePlanState : public yasmin::State {

public:
  ValidatePlanState()
      : yasmin::State({
            omni_plan::states::outcomes::VALID,
            omni_plan::states::outcomes::INVALID,
            yasmin_ros::basic_outcomes::ABORT,
        }) {
    this->set_description("Validate the generated plan using the plan "
                          "validator plugin.");
    this->set_outcome_description(omni_plan::states::outcomes::VALID,
                                  "The plan is valid.");
    this->set_outcome_description(omni_plan::states::outcomes::INVALID,
                                  "The plan is invalid.");
    this->set_outcome_description(
        yasmin_ros::basic_outcomes::ABORT,
        "Failed to validate the plan due to an infrastructure error.");
    this->add_input_key("plan_validator", "The plan validator plugin.");
    this->add_input_key("domain", "The PDDL domain.");
    this->add_input_key("problem", "The PDDL problem.");
    this->add_input_key("plan", "The PDDL plan.");
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) {
    PROFILE_FUNCTION_WITH_CHILDREN();

    try {
      auto plan_validator =
          blackboard->get<std::shared_ptr<omni_plan::PlanValidator>>(
              "plan_validator");

      if (!plan_validator) {
        YASMIN_LOG_ERROR("plan_validator is null");
        return yasmin_ros::basic_outcomes::ABORT;
      }

      if (!plan_validator->validate_plan(
              blackboard->get<omni_plan::pddl::Domain>("domain"),
              blackboard->get<omni_plan::pddl::Problem>("problem"),
              blackboard->get<omni_plan::pddl::Plan>("plan"))) {
        return omni_plan::states::outcomes::INVALID;
      }

    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to validate plan: %s", e.what());
      return yasmin_ros::basic_outcomes::ABORT;
    }

    return omni_plan::states::outcomes::VALID;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ValidatePlanState, yasmin::State)
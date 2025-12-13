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

#include <yasmin/state.hpp>

#include "easy_plan/plan_validator.hpp"
#include "easy_plan/states/outcomes.hpp"

class ValidatePlanState : public yasmin::State {

public:
  ValidatePlanState()
      : yasmin::State({
            easy_plan::states::outcomes::VALID,
            easy_plan::states::outcomes::INVALID,
        }) {}

  std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) {
    auto plan_validator =
        blackboard->get<std::shared_ptr<easy_plan::PlanValidator>>(
            "plan_validator");

    if (!plan_validator->validate_plan(
            blackboard->get<std::string>("domain"),
            blackboard->get<std::string>("problem"),
            blackboard->get<easy_plan::Plan>("plan"))) {
      return easy_plan::states::outcomes::INVALID;
    }

    return easy_plan::states::outcomes::VALID;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ValidatePlanState, yasmin::State)
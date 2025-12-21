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

#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

#include "easy_plan/pddl/plan.hpp"
#include "easy_plan/planner.hpp"

class PlanState : public yasmin::State {

public:
  PlanState()
      : yasmin::State({
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::ABORT,
        }) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) {
    auto planner =
        blackboard->get<std::shared_ptr<easy_plan::Planner>>("planner");
    blackboard->set<easy_plan::pddl::Plan>(
        "plan",
        planner->get_plan(
            blackboard->get<std::string>("domain"),
            blackboard->get<std::string>("problem"),
            blackboard->get<std::map<std::string,
                                     std::shared_ptr<easy_plan::pddl::Action>>>(
                "actions")));

    if (!blackboard->get<easy_plan::pddl::Plan>("plan").has_solution()) {
      YASMIN_LOG_WARN("Planner could not find a valid plan");
      return yasmin_ros::basic_outcomes::ABORT;
    }

    return yasmin_ros::basic_outcomes::SUCCEED;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PlanState, yasmin::State)
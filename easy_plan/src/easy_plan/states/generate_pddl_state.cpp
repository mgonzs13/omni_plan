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

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan/pddl_manager.hpp"

class GeneratePddlState : public yasmin::State {

public:
  GeneratePddlState() : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) {
    auto pddl_manager =
        blackboard->get<std::shared_ptr<easy_plan::PddlManager>>(
            "pddl_manager");
    auto actions_and_params = blackboard->get<std::unordered_map<
        std::string, std::shared_ptr<easy_plan::pddl::Action>>>("actions");

    std::vector<std::shared_ptr<easy_plan::pddl::Action>> actions;
    for (const auto &action_pair : actions_and_params) {
      actions.push_back(action_pair.second);
    }

    auto [domain, problem] = pddl_manager->get_pddl(actions);

    blackboard->set<easy_plan::pddl::Domain>("domain", domain);
    blackboard->set<easy_plan::pddl::Problem>("problem", problem);

    YASMIN_LOG_INFO("PDDL domain generated:\n%s", domain.to_pddl().c_str());
    YASMIN_LOG_INFO("PDDL problem generated:\n%s", problem.to_pddl().c_str());
    return yasmin_ros::basic_outcomes::SUCCEED;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GeneratePddlState, yasmin::State)
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

#include <yasmin/state.hpp>

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl_manager.hpp"
#include "easy_plan/states/outcomes.hpp"

class GeneratePddlState : public yasmin::State {

public:
  GeneratePddlState() : yasmin::State({easy_plan::states::outcomes::SUCCEED}) {}

  std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) {
    auto pddl_manager =
        blackboard->get<std::shared_ptr<easy_plan::PddlManager>>(
            "pddl_manager");
    auto actions = blackboard->get<
        std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>>>(
        "actions");

    std::vector<std::string> actions_pddl;
    for (const auto &action_pair : actions) {
      actions_pddl.push_back(action_pair.second->to_pddl());
    }

    auto [domain, problem] = pddl_manager->get_pddl(actions_pddl);
    blackboard->set<std::string>("domain", domain);
    blackboard->set<std::string>("problem", problem);

    YASMIN_LOG_INFO("PDDL domain generated:\n%s", domain.c_str());
    YASMIN_LOG_INFO("PDDL problem generated:\n%s", problem.c_str());

    return easy_plan::states::outcomes::SUCCEED;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GeneratePddlState, yasmin::State)
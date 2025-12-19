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
#include "easy_plan/pddl/expression.hpp"
#include "easy_plan/pddl_manager.hpp"

class GeneratePddlState : public yasmin::State {

public:
  GeneratePddlState() : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {}

  std::set<std::string> get_actions_types(
      std::vector<std::shared_ptr<easy_plan::pddl::Action>> actions) {
    std::set<std::string> types;

    for (const auto &action : actions) {
      auto params = action->get_parameters();
      for (const auto &param : params) {
        types.insert(param.type);
      }
    }
    return types;
  }

  std::string
  convert_action_predicate(std::shared_ptr<easy_plan::pddl::Predicate> pred,
                           std::shared_ptr<easy_plan::pddl::Action> action) {
    auto args = pred->get_args();
    std::string result = "(" + pred->get_name();
    std::string type;

    // Get types from action parameters
    if (args.size() == 1) {
      type = action->get_parameter_type(args[0]);
      result += " ?" + std::string(1, type[0]) + "0 - " + type;
    } else {
      for (size_t i = 0; i < args.size(); ++i) {
        type = action->get_parameter_type(args[i]);
        result +=
            " ?" + std::string(1, type[0]) + std::to_string(i) + " - " + type;
      }
    }

    result += ")";

    return result;
  }

  std::set<std::string> get_actions_predicates(
      std::vector<std::shared_ptr<easy_plan::pddl::Action>> actions) {

    std::set<std::string> actions_predicates;

    for (const auto &action : actions) {
      for (const auto &cond : action->get_conditions()) {
        actions_predicates.insert(
            this->convert_action_predicate(cond.expression, action));
      }
      for (const auto &eff : action->get_effects()) {
        actions_predicates.insert(
            this->convert_action_predicate(eff.expression, action));
      }
    }
    return actions_predicates;
  }

  std::set<std::string> get_actions_pddl(
      std::vector<std::shared_ptr<easy_plan::pddl::Action>> actions) {
    std::set<std::string> actions_pddl;
    for (const auto &action : actions) {
      actions_pddl.insert(action->to_pddl());
    }
    return actions_pddl;
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) {
    auto pddl_manager =
        blackboard->get<std::shared_ptr<easy_plan::PddlManager>>(
            "pddl_manager");
    auto actions_and_params = blackboard->get<
        std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>>>(
        "actions");

    std::vector<std::shared_ptr<easy_plan::pddl::Action>> actions;
    for (const auto &action_pair : actions_and_params) {
      actions.push_back(action_pair.second);
    }

    auto [domain, problem] = pddl_manager->get_pddl(
        this->get_actions_types(actions), this->get_actions_predicates(actions),
        this->get_actions_pddl(actions));
    blackboard->set<std::string>("domain", domain);
    blackboard->set<std::string>("problem", problem);

    YASMIN_LOG_INFO("PDDL domain generated:\n%s", domain.c_str());
    YASMIN_LOG_INFO("PDDL problem generated:\n%s", problem.c_str());

    return yasmin_ros::basic_outcomes::SUCCEED;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GeneratePddlState, yasmin::State)
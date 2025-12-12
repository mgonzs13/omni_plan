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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <yasmin/state.hpp>

#include "easy_plan/action.hpp"
#include "easy_plan/pddl_generator.hpp"
#include "easy_plan/plan_validator.hpp"
#include "easy_plan/planner.hpp"
#include "easy_plan/states/outcomes.hpp"

class LoadPluginsState : public yasmin::State {

public:
  LoadPluginsState()
      : yasmin::State({
            easy_plan::states::outcomes::SUCCEED,
            easy_plan::states::outcomes::FAILED,
        }) {}

  std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) {

    // Load PddlGenerator plugin
    pluginlib::ClassLoader<easy_plan::PddlGenerator>
        pddl_generator_state_loader_("easy_plan", "PddlGenerator");
    std::string pddl_generator_plugin =
        blackboard->get<std::string>("pddl_generator_plugin");

    try {
      blackboard->set<std::shared_ptr<easy_plan::PddlGenerator>>(
          "pddl_generator", pddl_generator_state_loader_.createSharedInstance(
                                pddl_generator_plugin));
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to load PddlGenerator plugin '%s': %s",
                       pddl_generator_plugin.c_str(), e.what());
      return easy_plan::states::outcomes::FAILED;
    }

    // Load Planner plugin
    pluginlib::ClassLoader<easy_plan::Planner> planner_state_loader_(
        "easy_plan", "Planner");
    std::string planner_plugin = blackboard->get<std::string>("planner_plugin");

    try {
      blackboard->set<std::shared_ptr<easy_plan::Planner>>(
          "planner",
          planner_state_loader_.createSharedInstance(planner_plugin));
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to load Planner plugin '%s': %s",
                       planner_plugin.c_str(), e.what());
      return easy_plan::states::outcomes::FAILED;
    }

    // Load PlanValidator plugin
    pluginlib::ClassLoader<easy_plan::PlanValidator>
        plan_validator_state_loader_("easy_plan", "PlanValidator");
    std::string plan_validator_plugin =
        blackboard->get<std::string>("plan_validator_plugin");

    try {
      blackboard->set<std::shared_ptr<easy_plan::PlanValidator>>(
          "plan_validator", plan_validator_state_loader_.createSharedInstance(
                                plan_validator_plugin));
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to load PlanValidator plugin '%s': %s",
                       plan_validator_plugin.c_str(), e.what());
      return easy_plan::states::outcomes::FAILED;
    }

    // Load Action plugins
    pluginlib::ClassLoader<easy_plan::Action> action_state_loader_("easy_plan",
                                                                   "Action");
    // std::vector<std::string> action_plugins =
    //     blackboard->get<std::vector<std::string>>("action_plugins");

    // std::map<std::string, std::shared_ptr<easy_plan::Action>> actions;
    // for (const auto &action_plugin : action_plugins) {
    //   try {
    //     auto action =
    //     action_state_loader_.createSharedInstance(action_plugin);
    //     actions[action->get_name()] = action;
    //   } catch (const std::exception &e) {
    //     YASMIN_LOG_ERROR("Failed to create Action plugin instance '%s': %s",
    //                      action_plugin.c_str(), e.what());
    //     return easy_plan::states::outcomes::FAILED;
    //   }
    // }
    // blackboard->set<std::map<std::string,
    // std::shared_ptr<easy_plan::Action>>>(
    //     "actions", actions);

    return easy_plan::states::outcomes::SUCCEED;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LoadPluginsState, yasmin::State)
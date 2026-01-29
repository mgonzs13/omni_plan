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

#include "omni_plan/pddl_manager.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/utils/parameter_loader.hpp"

using namespace omni_plan;

PddlManager::PddlManager() : utils::ParameterLoader("pddl_manager") {

  this->add_ros_parameters(
      {{"domain_requirements",
        std::vector<std::string>{"typing", "negative-preconditions",
                                 "durative-actions"},
        this->domain_requirements}});
}

std::pair<pddl::Domain, pddl::Problem> PddlManager::get_pddl(
    std::vector<std::shared_ptr<pddl::Action>> actions) const {

  auto [domain, problem] = this->get_pddl();

  // Add requirements to domain
  for (const auto &req : this->domain_requirements) {
    domain.add_requirement(req);
  }

  // Add actions to domain
  for (const auto &action : actions) {
    domain.add_action(action);
    auto action_types = this->get_actions_types(action);
    for (const auto &type : action_types) {
      domain.add_type(type);
    }

    auto action_predicates = this->get_action_predicates(action);
    for (const auto &pred : action_predicates) {
      if (!pred.is_negated()) {
        domain.add_predicate(pddl::Predicate(pred));
      }
    }
  }

  return std::make_pair(domain, problem);
}

std::vector<pddl::Effect>
PddlManager::apply_effects(const std::vector<pddl::Effect> &effects) {

  std::vector<pddl::Effect> applied_effects;

  for (const auto &effect : effects) {
    if ((!this->predicate_exists(effect) && !effect.is_negated()) or
        (this->predicate_exists(effect) || effect.is_negated())) {
      this->apply_effect(effect);
      applied_effects.push_back(effect);
    }
  }

  return applied_effects;
}

std::set<std::string> PddlManager::get_actions_types(
    std::shared_ptr<omni_plan::pddl::Action> action) const {
  std::set<std::string> types;

  auto params = action->get_parameters();
  for (const auto &param : params) {
    types.insert(param.get_type());
  }

  return types;
}

omni_plan::pddl::Predicate PddlManager::convert_action_predicate(
    omni_plan::pddl::Predicate pred,
    std::shared_ptr<omni_plan::pddl::Action> action) const {

  std::vector<std::string> args = pred.get_args();
  std::vector<std::string> new_args;
  std::string type;

  // Get types from action parameters
  for (size_t i = 0; i < args.size(); ++i) {
    type = action->get_parameter_type(args[i]);
    new_args.push_back(type);
  }

  omni_plan::pddl::Predicate predicate(pred.get_name(), new_args,
                                       pred.is_negated());
  return predicate;
}

std::set<omni_plan::pddl::Predicate> PddlManager::get_action_predicates(
    std::shared_ptr<omni_plan::pddl::Action> action) const {

  std::set<omni_plan::pddl::Predicate> predicates;

  for (const auto &cond : action->get_conditions()) {
    predicates.insert(this->convert_action_predicate(cond, action));
  }
  for (const auto &eff : action->get_effects()) {
    predicates.insert(this->convert_action_predicate(eff, action));
  }

  return predicates;
}

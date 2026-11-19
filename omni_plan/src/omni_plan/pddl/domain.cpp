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

#include "omni_plan_msgs/msg/domain.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"

using namespace omni_plan::pddl;

void Domain::add_requirement(const std::string &requirement) {
  this->requirements_.insert(requirement);
}

void Domain::add_type(const std::string &type) { this->types_.insert(type); }

void Domain::add_predicate(const Predicate &pred) {
  this->predicates_.insert(pred);
}

void Domain::add_action(const std::shared_ptr<Action> &action) {
  this->actions_[action->get_name()] = action;
}

std::string Domain::to_pddl() const {
  std::string pddl = "(define (domain omni_plan_domain)\n\n";

  // Requirements
  pddl += "(:requirements";

  for (const auto &req : this->requirements_) {
    pddl += " :" + req;
  }

  pddl += ")\n\n";

  // Types
  if (!types_.empty()) {
    pddl += "(:types\n";
    for (const auto &type : this->types_) {
      pddl += "  " + type + "\n";
    }
    pddl += ")\n\n";
  }

  // Predicates
  if (!predicates_.empty()) {
    pddl += "(:predicates\n";
    for (const auto &pred : this->predicates_) {
      pddl += "  " + pred.to_pddl() + "\n";
    }
    pddl += ")\n\n";
  }

  // Actions (iterated in key order via std::map)
  for (const auto &pair : this->actions_) {
    pddl += pair.second->to_pddl() + "\n";
  }

  pddl += ")";

  return pddl;
}

omni_plan_msgs::msg::Domain Domain::to_msg() const {
  omni_plan_msgs::msg::Domain msg;

  for (const auto &req : this->requirements_) {
    msg.requirements.push_back(req);
  }

  for (const auto &type : this->types_) {
    msg.types.push_back(type);
  }

  for (const auto &pred : this->predicates_) {
    msg.predicates.push_back(pred.to_msg());
  }

  for (const auto &pair : this->actions_) {
    msg.actions.push_back(pair.second->to_msg());
  }

  return msg;
}
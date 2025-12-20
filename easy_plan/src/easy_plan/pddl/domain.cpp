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

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/expression.hpp"
#include "easy_plan/pddl/object.hpp"

using namespace easy_plan::pddl;

void Domain::add_type(const std::string &type) { this->types_.insert(type); }

void Domain::add_predicate(const Predicate &pred) {
  this->predicates_.insert(pred);
}

void Domain::add_action(const std::shared_ptr<Action> &action) {
  this->actions_.insert(action);
}

std::string Domain::to_pddl() const {
  std::string pddl = "(define (domain easy_plan_domain)\n\n";
  pddl +=
      "(:requirements :typing :durative-actions :negative-preconditions)\n\n ";

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

  // Actions
  for (const auto &action : this->actions_) {
    pddl += action->to_pddl() + "\n";
  }

  pddl += ")";

  return pddl;
}
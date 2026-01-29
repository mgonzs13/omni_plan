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

#include "omni_plan_msgs/msg/problem.hpp"

#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"

using namespace omni_plan::pddl;

void Problem::add_object(const Object &obj) { this->objects_.insert(obj); }

void Problem::add_goal(const Predicate &goal) { this->goals_.insert(goal); }

void Problem::add_fact(const Predicate &fact) { this->facts_.insert(fact); }

std::string Problem::to_pddl() const {
  std::string pddl = "(define (problem knowledge_graph_problem)\n\n";

  pddl += "(:domain omni_plan_domain)\n\n";

  // Objects
  pddl += "(:objects\n";
  for (const auto &obj : this->objects_) {
    pddl += "  " + obj.to_pddl() + "\n";
  }
  pddl += ")\n\n";

  // Init
  pddl += "(:init\n";
  for (const auto &fact : this->facts_) {
    pddl += "  " + fact.to_pddl(true) + "\n";
  }
  pddl += ")\n\n";

  // Goal
  pddl += "(:goal\n  (and\n";
  for (const auto &goal : this->goals_) {
    pddl += "    " + goal.to_pddl(true) + "\n";
  }
  pddl += "  )\n)\n";

  pddl += ")";

  return pddl;
}

omni_plan_msgs::msg::Problem Problem::to_msg() const {
  omni_plan_msgs::msg::Problem msg;
  for (const auto &obj : this->objects_) {
    msg.objects.push_back(obj.to_msg());
  }
  for (const auto &goal : this->goals_) {
    msg.goals.push_back(goal.to_msg());
  }
  for (const auto &fact : this->facts_) {
    msg.facts.push_back(fact.to_msg());
  }
  return msg;
}
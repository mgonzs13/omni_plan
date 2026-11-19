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

#include "omni_plan_msgs/msg/problem.hpp"

#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"

using namespace omni_plan::pddl;

void Problem::add_object(const Object &obj) { this->objects_.insert(obj); }

void Problem::add_goal(const Predicate &goal) { this->goals_.insert(goal); }

void Problem::add_fact(const Predicate &fact) { this->facts_.insert(fact); }

std::string Problem::to_pddl() const {
  std::string pddl = "(define (problem omni_plan_problem)\n\n";

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
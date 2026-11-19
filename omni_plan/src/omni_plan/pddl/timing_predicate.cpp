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

#include "omni_plan_msgs/msg/predicate.hpp"

#include "omni_plan/pddl/timing_predicate.hpp"

using namespace omni_plan::pddl;

TimingPredicate::TimingPredicate(Type type, const std::string &name,
                                 const std::vector<std::string> &args,
                                 bool negated)
    : Predicate(name, args, negated), type_(type) {}

Type TimingPredicate::get_type() const { return this->type_; }

std::string TimingPredicate::to_pddl(bool as_fact) const {
  (void)as_fact; // Unused parameter

  std::string timing_str;
  switch (type_) {
  case START:
    timing_str = "at start";
    break;
  case OVER_ALL:
    timing_str = "over all";
    break;
  case END:
    timing_str = "at end";
    break;
  }

  std::string s = "(" + timing_str;

  if (this->is_negated()) {
    s += " (not";
  }

  s += " (" + this->get_name();

  for (const auto &arg : this->get_args()) {
    // Accept both "r" and "?r" parameter names and always render "?r".
    std::string name = arg;
    if (!name.empty() && name.front() == '?') {
      name.erase(0, 1);
    }
    s += " ?" + name;
  }

  if (this->is_negated()) {
    s += ")";
  }

  s += "))";

  return s;
}

omni_plan_msgs::msg::Predicate TimingPredicate::to_msg() const {
  omni_plan_msgs::msg::Predicate msg = Predicate::to_msg();
  switch (this->type_) {
  case START:
    msg.time = omni_plan_msgs::msg::Predicate::AT_START;
    break;
  case OVER_ALL:
    msg.time = omni_plan_msgs::msg::Predicate::OVER_ALL;
    break;
  case END:
    msg.time = omni_plan_msgs::msg::Predicate::AT_END;
    break;
  }
  return msg;
}
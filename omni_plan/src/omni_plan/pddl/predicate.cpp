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

#include "omni_plan/pddl/predicate.hpp"

using namespace omni_plan::pddl;

Predicate::Predicate(const std::string &name,
                     const std::vector<std::string> &args, bool negated)
    : name_(name), args_(args), negated_(negated) {}

std::string Predicate::get_name() const { return this->name_; }

std::vector<std::string> Predicate::get_args() const { return this->args_; }

bool Predicate::is_negated() const { return this->negated_; }

void Predicate::set_negation(bool negated) { this->negated_ = negated; }

std::string Predicate::to_pddl(bool as_fact) const {
  std::string s = "(" + this->get_name();
  for (size_t i = 0; i < this->args_.size(); i++) {
    auto arg = this->args_.at(i);

    if (as_fact) {
      s += " " + arg;
    } else {
      // Type names must not already carry the PDDL variable marker, otherwise
      // the emitted variable would be "??x".
      std::string type_name =
          (!arg.empty() && arg[0] == '?') ? arg.substr(1) : arg;
      if (!type_name.empty()) {
        s += " ?" + std::string(1, type_name[0]) + std::to_string(i) + " - " +
             type_name;
      }
    }
  }
  s += ")";
  if (this->negated_)
    s = "(not " + s + ")";
  return s;
}

omni_plan_msgs::msg::Predicate Predicate::to_msg() const {
  omni_plan_msgs::msg::Predicate msg;
  msg.name = this->name_;
  msg.arguments = this->args_;
  msg.negated = this->negated_;
  return msg;
}

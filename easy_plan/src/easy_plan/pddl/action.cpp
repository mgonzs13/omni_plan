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

#include <set>

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/timing_predicate.hpp"
#include "easy_plan/utils/parameter_loader.hpp"

using namespace easy_plan::pddl;

Action::Action(const std::string &name,
               const std::vector<std::pair<std::string, std::string>> &params)
    : utils::ParameterLoader(name + "_action"), name_(name) {
  for (const auto &param : params) {
    this->parameters_.emplace_back(Parameter(param.first, param.second));
  }
}

std::string Action::get_name() const { return this->name_; }

void Action::add_condition(Type type, std::string name,
                           const std::vector<std::string> &args, bool negated) {
  this->conditions_.push_back(TimingPredicate(type, name, args, negated));
}

void Action::add_effect(Type type, std::string name,
                        const std::vector<std::string> &args, bool negated) {
  this->effects_.push_back(TimingPredicate(type, name, args, negated));
}

std::vector<Parameter> Action::get_parameters() const {
  return this->parameters_;
}

std::string Action::get_parameter_type(const std::string &param_name) const {
  for (const auto &param : this->parameters_) {
    if (param.get_name() == param_name) {
      return param.get_type();
    }
  }
  return "unknown_type";
}

int Action::get_parameter_index(const std::string &param_name) const {
  for (size_t i = 0; i < this->parameters_.size(); ++i) {
    if (this->parameters_[i].get_name() == param_name) {
      return static_cast<int>(i);
    }
  }
  return -1; // Not found
}

std::vector<Condition> Action::get_conditions() const {
  return this->conditions_;
}

std::vector<Condition> Action::get_on_start_conditions() const {
  std::vector<Condition> on_start_conditions;
  for (const auto &condition : this->conditions_) {
    if (condition.get_type() == START) {
      on_start_conditions.push_back(condition);
    }
  }
  return on_start_conditions;
}

std::vector<Condition> Action::get_on_end_conditions() const {
  std::vector<Condition> on_end_conditions;
  for (const auto &condition : this->conditions_) {
    if (condition.get_type() == END) {
      on_end_conditions.push_back(condition);
    }
  }
  return on_end_conditions;
}

std::vector<Condition> Action::get_over_all_conditions() const {
  std::vector<Condition> over_all_conditions;
  for (const auto &condition : this->conditions_) {
    if (condition.get_type() == OVER_ALL) {
      over_all_conditions.push_back(condition);
    }
  }
  return over_all_conditions;
}

std::vector<Effect> Action::get_effects() const { return this->effects_; }

std::vector<Effect> Action::get_on_start_effects() const {
  std::vector<Effect> on_start_effects;
  for (const auto &effect : this->effects_) {
    if (effect.get_type() == START) {
      on_start_effects.push_back(effect);
    }
  }
  return on_start_effects;
}

std::vector<Effect> Action::get_on_end_effects() const {
  std::vector<Effect> on_end_effects;
  for (const auto &effect : this->effects_) {
    if (effect.get_type() == END) {
      on_end_effects.push_back(effect);
    }
  }
  return on_end_effects;
}

std::vector<Effect> Action::get_over_all_effects() const {
  std::vector<Effect> over_all_effects;
  for (const auto &effect : this->effects_) {
    if (effect.get_type() == OVER_ALL) {
      over_all_effects.push_back(effect);
    }
  }
  return over_all_effects;
}

std::string
Action::build_timing_section(const std::string &section,
                             const std::vector<TimingPredicate> &items) const {
  std::string s = "  :" + section + " (and ";
  for (const auto &item : items) {
    s += item.to_pddl(true) + " ";
  }
  s += ")\n";
  return s;
}

std::string Action::to_pddl() const {
  std::string pddl = "(:durative-action " + this->name_ + "\n";
  pddl += "  :parameters (";
  for (size_t i = 0; i < this->parameters_.size(); ++i) {
    pddl += "?" + this->parameters_[i].get_name() + " - " +
            this->parameters_[i].get_type();
    if (i < this->parameters_.size() - 1)
      pddl += " ";
  }
  pddl += ")\n";
  pddl += "  :duration (= ?duration 10)\n";
  pddl += this->build_timing_section("condition", this->conditions_);
  pddl += this->build_timing_section("effect", this->effects_);
  pddl += ")";
  return pddl;
}

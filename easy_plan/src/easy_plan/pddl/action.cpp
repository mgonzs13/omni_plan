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

#include "easy_plan/pddl/action.hpp"

#include <set>

namespace easy_plan {
namespace pddl {

std::string Action::get_timing(TimingExpression::Type type) const {
  switch (type) {
  case TimingExpression::START:
    return "at start";
  case TimingExpression::OVER_ALL:
    return "over all";
  case TimingExpression::END:
    return "at end";
  }
  return "";
}

std::string
Action::build_timing_section(const std::string &section,
                             const std::vector<TimingExpression> &items) const {
  std::string s = "  :" + section + " (and";
  for (const auto &item : items) {
    s += " (" + get_timing(item.type) + " " + item.expression->to_pddl() + ")";
  }
  s += ")\n";
  return s;
}

std::string Action::to_pddl() const {
  std::string pddl = "(:durative-action " + this->name_ + "\n";
  pddl += "  :parameters (";
  for (size_t i = 0; i < this->parameters_.size(); ++i) {
    pddl += "?" + this->parameters_[i].name + " - " + this->parameters_[i].type;
    if (i < this->parameters_.size() - 1)
      pddl += " ";
  }
  pddl += ")\n";
  pddl += "  :duration (= ?duration 10)\n";
  pddl += build_timing_section("condition", this->conditions_);
  pddl += build_timing_section("effect", this->effects_);
  pddl += ")";
  return pddl;
}

bool Action::validate_pddl() const {
  // Collect all parameter names with ?
  std::set<std::string> param_names;
  for (const auto &param : this->parameters_) {
    param_names.insert("?" + param.name);
  }

  // Check conditions
  for (const auto &cond : this->conditions_) {
    const auto &args = cond.expression->get_args();
    for (const auto &arg : args) {
      if (param_names.find(arg) == param_names.end()) {
        return false;
      }
    }
  }

  // Check effects
  for (const auto &eff : this->effects_) {
    const auto &args = eff.expression->get_args();
    for (const auto &arg : args) {
      if (param_names.find(arg) == param_names.end()) {
        return false;
      }
    }
  }

  return true;
}

Action::Action(const std::string &name, const std::vector<Parameter> &params)
    : name_(name), parameters_(params) {}

std::string Action::get_name() const { return this->name_; }

void Action::add_condition(Condition::Type type,
                           std::shared_ptr<Predicate> pred) {
  this->conditions_.push_back({type, pred});
}

void Action::add_effect(Effect::Type type, std::shared_ptr<Predicate> pred) {
  this->effects_.push_back({type, pred});
}

std::vector<Parameter> Action::get_parameters() const {
  return this->parameters_;
}

std::vector<Condition> Action::get_conditions() const {
  return this->conditions_;
}

std::vector<Condition> Action::get_on_start_conditions() const {
  std::vector<Condition> on_start_conditions;
  for (const auto &condition : this->conditions_) {
    if (condition.type == TimingExpression::START) {
      on_start_conditions.push_back(condition);
    }
  }
  return on_start_conditions;
}

std::vector<Condition> Action::get_on_end_conditions() const {
  std::vector<Condition> on_end_conditions;
  for (const auto &condition : this->conditions_) {
    if (condition.type == TimingExpression::END) {
      on_end_conditions.push_back(condition);
    }
  }
  return on_end_conditions;
}

std::vector<Condition> Action::get_over_all_conditions() const {
  std::vector<Condition> over_all_conditions;
  for (const auto &condition : this->conditions_) {
    if (condition.type == TimingExpression::OVER_ALL) {
      over_all_conditions.push_back(condition);
    }
  }
  return over_all_conditions;
}

std::vector<Effect> Action::get_effects() const { return this->effects_; }

std::vector<Effect> Action::get_on_start_effects() const {
  std::vector<Effect> on_start_effects;
  for (const auto &effect : this->effects_) {
    if (effect.type == TimingExpression::START) {
      on_start_effects.push_back(effect);
    }
  }
  return on_start_effects;
}

std::vector<Effect> Action::get_on_end_effects() const {
  std::vector<Effect> on_end_effects;
  for (const auto &effect : this->effects_) {
    if (effect.type == TimingExpression::END) {
      on_end_effects.push_back(effect);
    }
  }
  return on_end_effects;
}

std::vector<Effect> Action::get_over_all_effects() const {
  std::vector<Effect> over_all_effects;
  for (const auto &effect : this->effects_) {
    if (effect.type == TimingExpression::OVER_ALL) {
      over_all_effects.push_back(effect);
    }
  }
  return over_all_effects;
}

} // namespace pddl
} // namespace easy_plan
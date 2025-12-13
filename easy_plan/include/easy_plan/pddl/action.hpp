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

#ifndef EASY_PLAN__ACTION_HPP__
#define EASY_PLAN__ACTION_HPP__

#include <memory>
#include <string>
#include <vector>

#include "easy_plan/pddl/expression.hpp"

namespace easy_plan {
namespace pddl {

class Action {
public:
  Action(const std::string &name, const std::vector<Parameter> &params = {})
      : name_(name), parameters_(params) {};

  virtual ~Action() = default;

  std::string get_name() const { return this->name_; };

  void add_condition(Condition::Type type, std::unique_ptr<Expression> expr) {
    conditions_.push_back({type, std::move(expr)});
  };

  void add_effect(Effect::Type type, std::unique_ptr<Expression> expr) {
    effects_.push_back({type, std::move(expr)});
  };

private:
  std::string get_timing(TimingExpression::Type type) const {
    switch (type) {
    case TimingExpression::START:
      return "at start";
    case TimingExpression::OVER_ALL:
      return "over all";
    case TimingExpression::END:
      return "at end";
    }
    return "";
  };

  std::string
  build_timing_section(const std::string &section,
                       const std::vector<TimingExpression> &items) const {
    std::string s = "  :" + section + " (and";
    for (const auto &item : items) {
      s += " (" + get_timing(item.type) + " " + item.expression->to_string() +
           ")";
    }
    s += ")\n";
    return s;
  };

public:
  std::string to_pddl() const {
    std::string pddl = "(:durative-action " + this->name_ + "\n";
    pddl += "  :parameters (";
    for (size_t i = 0; i < this->parameters_.size(); ++i) {
      pddl +=
          "?" + this->parameters_[i].name + " - " + this->parameters_[i].type;
      if (i < this->parameters_.size() - 1)
        pddl += " ";
    }
    pddl += ")\n";
    pddl += "  :duration (= ?duration 10)\n";
    pddl += build_timing_section("condition", this->conditions_);
    pddl += build_timing_section("effect", this->effects_);
    pddl += ")";
    return pddl;
  };

  virtual void run(std::vector<std::string> params) = 0;

  virtual void cancel() = 0;

private:
  std::string name_;
  std::vector<Parameter> parameters_;
  std::vector<Condition> conditions_;
  std::vector<Effect> effects_;
};

} // namespace pddl
} // namespace easy_plan
#endif // EASY_PLAN__ACTION_HPP__
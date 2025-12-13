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

#ifndef EASY_PLAN__EXPRESSION_HPP__
#define EASY_PLAN__EXPRESSION_HPP__

#include <memory>
#include <string>
#include <vector>

namespace easy_plan {
namespace pddl {

class Expression {
public:
  virtual ~Expression() = default;
  virtual std::string to_pddl() const = 0;
};

class Predicate : public Expression {
public:
  Predicate(const std::string &name, const std::vector<std::string> &args = {})
      : name_(name), args_(args) {}

  std::string get_name() const { return this->name_; }

  std::vector<std::string> get_args() const { return this->args_; }

  std::string to_pddl() const override {
    std::string s = "(" + this->name_;
    for (const auto &arg : this->args_)
      s += " " + arg;
    s += ")";
    return s;
  }

private:
  std::string name_;
  std::vector<std::string> args_;
};

class Not : public Expression {
public:
  Not(std::shared_ptr<Expression> expr) : expr_(expr) {}

  std::string to_pddl() const override {
    return "(not " + this->expr_->to_pddl() + ")";
  }

private:
  std::shared_ptr<Expression> expr_;
};

struct Object {
  std::string name;
  std::string type;
};

using Parameter = Object;

struct TimingExpression {
  enum Type { START, OVER_ALL, END };
  Type type;
  std::shared_ptr<Expression> expression;
};

using Condition = TimingExpression;
using Effect = TimingExpression;

} // namespace pddl
} // namespace easy_plan
#endif // EASY_PLAN__EXPRESSION_HPP__
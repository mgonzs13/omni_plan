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

#ifndef EASY_PLAN__EXPRESSION_HPP_
#define EASY_PLAN__EXPRESSION_HPP_

#include <memory>
#include <string>
#include <vector>

namespace easy_plan {
namespace pddl {

class Predicate {
public:
  Predicate(const std::string &name, const std::vector<std::string> &args = {},
            bool negated = false);

  std::string get_name() const;

  std::vector<std::string> get_args() const;

  bool is_negated() const;

  void set_negation(bool negated);

  std::string to_pddl() const;

private:
  std::string name_;
  std::vector<std::string> args_;
  bool negated_;
};

struct TimingExpression {
  enum Type { START, OVER_ALL, END };
  Type type;
  Predicate expression;
};

using Condition = TimingExpression;
using Effect = TimingExpression;

} // namespace pddl
} // namespace easy_plan

#endif // EASY_PLAN__EXPRESSION_HPP_

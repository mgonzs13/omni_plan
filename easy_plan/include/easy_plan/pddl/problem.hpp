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

#ifndef EASY_PLAN__PROBLEM_HPP_
#define EASY_PLAN__PROBLEM_HPP_

#include <memory>
#include <set>
#include <string>

#include "easy_plan/pddl/expression.hpp"
#include "easy_plan/pddl/object.hpp"

namespace easy_plan {
namespace pddl {

class Problem {
public:
  Problem() = default;

  void add_object(const Object &obj);

  void add_goal(const Predicate &goal);

  void add_fact(const Predicate &fact);

  std::string to_pddl() const;

private:
  std::set<Object> objects_;
  std::set<Predicate> goals_;
  std::set<Predicate> facts_;
};

} // namespace pddl
} // namespace easy_plan

#endif // EASY_PLAN__PROBLEM_HPP_

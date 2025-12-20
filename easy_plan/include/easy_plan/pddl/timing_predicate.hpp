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

#ifndef EASY_PLAN__TIMING_PREDICATE_HPP_
#define EASY_PLAN__TIMING_PREDICATE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "easy_plan/pddl/predicate.hpp"

namespace easy_plan {
namespace pddl {

class TimingPredicate : public Predicate {

public:
  enum Type { START, OVER_ALL, END };

  TimingPredicate(Type type, const std::string &name,
                  const std::vector<std::string> &args = {},
                  bool negated = false);

  Type get_type() const;

  std::string to_pddl(bool as_fact = false) const;

private:
  Type type_;
};

using Condition = TimingPredicate;

using Effect = TimingPredicate;

} // namespace pddl
} // namespace easy_plan

#endif // EASY_PLAN__TIMING_PREDICATE_HPP_

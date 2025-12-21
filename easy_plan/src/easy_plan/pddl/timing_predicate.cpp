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

#include "easy_plan/pddl/timing_predicate.hpp"

using namespace easy_plan::pddl;

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
    s += " ?" + arg;
  }

  if (this->is_negated()) {
    s += ")";
  }

  s += "))";

  return s;
}

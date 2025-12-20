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

#include "easy_plan/pddl/predicate.hpp"

using namespace easy_plan::pddl;

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
      s += " ?" + std::string(1, arg[0]) + std::to_string(i) + " - " + arg;
    }
  }
  s += ")";
  if (this->negated_)
    s = "(not " + s + ")";
  return s;
}

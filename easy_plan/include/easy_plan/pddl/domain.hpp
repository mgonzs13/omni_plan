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

#ifndef EASY_PLAN__DOMAIN_HPP_
#define EASY_PLAN__DOMAIN_HPP_

#include <memory>
#include <set>
#include <string>

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/predicate.hpp"

namespace easy_plan {
namespace pddl {

class Domain {
public:
  Domain() = default;

  void add_type(const std::string &type);

  void add_predicate(const Predicate &pred);

  void add_action(const std::shared_ptr<Action> &action);

  std::string to_pddl() const;

private:
  std::set<std::string> types_;
  std::set<Predicate> predicates_;
  std::set<std::shared_ptr<Action>> actions_;
};

} // namespace pddl
} // namespace easy_plan

#endif // EASY_PLAN__DOMAIN_HPP_

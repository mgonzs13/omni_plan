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

#ifndef EASY_PLAN__PDDL_MANAGER_HPP__
#define EASY_PLAN__PDDL_MANAGER_HPP__

#include <string>

#include "easy_plan/pddl/expression.hpp"

namespace easy_plan {

class PddlManager {
public:
  PddlManager() = default;

  virtual ~PddlManager() = default;

  virtual std::pair<std::string, std::string>
  get_pddl(std::vector<std::string> actions_pddl) const = 0;

  std::pair<std::string, std::string> get_pddl() const;

  virtual bool has_goals() const = 0;

  virtual void apply_effect(pddl::Effect exp) = 0;

  virtual void undo_effect(pddl::Effect exp) = 0;

  void apply_effects(const std::vector<pddl::Effect> &effects);

  void undo_effects(const std::vector<pddl::Effect> &effects);
};

} // namespace easy_plan
#endif // EASY_PLAN__PDDL_MANAGER_HPP__
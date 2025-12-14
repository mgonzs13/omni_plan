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

#include "easy_plan/pddl_manager.hpp"

using namespace easy_plan;

std::pair<std::string, std::string> PddlManager::get_pddl() const {
  return this->get_pddl({}, {}, {});
}

std::vector<pddl::Effect>
PddlManager::apply_effects(const std::vector<pddl::Effect> &effects) {

  std::vector<pddl::Effect> applied_effects;

  for (const auto &effect : effects) {
    if ((!this->predicate_exists(*effect.expression) &&
         !effect.expression->is_negated()) or
        (this->predicate_exists(*effect.expression) ||
         effect.expression->is_negated())) {
      this->apply_effect(effect);
      applied_effects.push_back(effect);
    }
  }

  return applied_effects;
}

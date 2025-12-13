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

#include "easy_plan/plan.hpp"

namespace easy_plan {

Plan::Plan(bool has_solution)
    : has_solution_(has_solution), actions_(), params_(), current_index_(0) {}

bool Plan::has_solution() const { return this->has_solution_; }

void Plan::add_action(const std::shared_ptr<pddl::Action> &action,
                      const std::vector<std::string> &params) {
  this->actions_.push_back(action);
  this->params_.push_back(params);
}

bool Plan::get_next_action(std::shared_ptr<pddl::Action> &action,
                           std::vector<std::string> &params) {
  if (this->current_index_ >= this->actions_.size()) {
    return false;
  }
  action = this->actions_[this->current_index_];
  params = this->params_[this->current_index_];
  this->current_index_++;
  return true;
}

} // namespace easy_plan
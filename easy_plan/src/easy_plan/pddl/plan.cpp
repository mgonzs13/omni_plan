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

#include "easy_plan/pddl/plan.hpp"

using namespace easy_plan::pddl;

Plan::Plan(bool has_solution)
    : has_solution_(has_solution), actions_(), params_(), current_index_(0) {}

bool Plan::has_solution() const { return this->has_solution_; }

void Plan::add_action(const std::shared_ptr<pddl::Action> &action,
                      const std::vector<std::string> &params) {
  this->actions_.push_back(action);
  this->params_.push_back(params);
}

size_t Plan::size() const { return this->actions_.size(); }

std::shared_ptr<Action> Plan::get_action(size_t index) const {
  return this->actions_.at(index);
}

std::vector<std::string> Plan::get_action_params(size_t index) const {
  return this->params_.at(index);
}

std::pair<std::shared_ptr<Action>, std::vector<std::string>>
Plan::get_action_with_params(size_t index) const {
  return std::make_pair(this->get_action(index),
                        this->get_action_params(index));
}

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

#include "omni_plan_msgs/msg/plan.hpp"
#include "omni_plan_msgs/msg/plan_action.hpp"

#include "omni_plan/pddl/plan.hpp"

using namespace omni_plan::pddl;

Plan::Plan(bool has_solution)
    : has_solution_(has_solution), actions_(), params_(), current_index_(0) {}

void Plan::set_has_solution(bool has_solution) {
  this->has_solution_ = has_solution;
}

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

std::string Plan::to_pddl() const {

  std::string pddl = "\n";

  for (size_t i = 0; i < this->actions_.size(); ++i) {
    pddl += "(" + this->actions_[i]->get_name();
    if (!this->params_[i].empty()) {
      pddl += " ";
      for (size_t j = 0; j < this->params_[i].size(); ++j) {
        pddl += this->params_[i][j];
        if (j < this->params_[i].size() - 1) {
          pddl += " ";
        }
      }
    }
    pddl += ")\n";
  }

  return pddl;
}

omni_plan_msgs::msg::Plan Plan::to_msg() const {
  omni_plan_msgs::msg::Plan msg;
  msg.has_solution = this->has_solution_;
  for (size_t i = 0; i < this->actions_.size(); ++i) {
    omni_plan_msgs::msg::PlanAction plan_action_msg;
    plan_action_msg.parameters = this->params_[i];
    plan_action_msg.action = this->actions_[i]->to_msg();
    msg.actions.push_back(plan_action_msg);
  }
  return msg;
}
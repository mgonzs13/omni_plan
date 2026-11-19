// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "omni_plan_msgs/msg/plan.hpp"
#include "omni_plan_msgs/msg/plan_action.hpp"

#include <utility>

#include "omni_plan/pddl/plan.hpp"

using namespace omni_plan::pddl;

Plan::Plan()
    : raw_output(""), has_solution_(false), actions_(), params_(),
      start_times_() {}

void Plan::set_raw_output(const std::string &raw_output) {
  this->raw_output = raw_output;
}

std::string Plan::get_raw_output() const { return this->raw_output; }

void Plan::set_has_solution(bool has_solution) {
  this->has_solution_ = has_solution;
}

bool Plan::has_solution() const { return this->has_solution_; }

void Plan::add_action(const std::shared_ptr<pddl::Action> &action,
                      const std::vector<std::string> &params,
                      float start_time) {
  this->actions_.push_back(action);
  this->params_.push_back(params);
  this->start_times_.push_back(start_time);
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

float Plan::get_action_start_time(size_t index) const {
  return this->start_times_.at(index);
}

std::string Plan::to_pddl() const {

  std::string pddl = "\n";

  for (size_t i = 0; i < this->actions_.size(); ++i) {
    pddl += std::to_string(this->start_times_[i]) + ": (";
    pddl += this->actions_[i]->get_name();
    if (!this->params_[i].empty()) {
      pddl += " ";
      for (size_t j = 0; j < this->params_[i].size(); ++j) {
        pddl += this->params_[i][j];
        if (j < this->params_[i].size() - 1) {
          pddl += " ";
        }
      }
    }
    pddl += ") [" + std::to_string(this->actions_[i]->get_duration()) + "]\n";
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

void Plan::set_planning_graph(std::shared_ptr<PlanningGraph> graph) {
  this->planning_graph_ = std::move(graph);
}

std::shared_ptr<PlanningGraph> Plan::get_planning_graph() const {
  return this->planning_graph_;
}

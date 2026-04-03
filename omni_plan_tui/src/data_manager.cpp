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

#include "omni_plan_tui/data_manager.hpp"

using namespace omni_plan_tui;

void DataManager::set_actions_info(
    const omni_plan_msgs::msg::ActionInfoArray::SharedPtr &msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->actions_info_ = *msg;
  this->has_actions_info_ = true;
}

void DataManager::set_plan_execution_status(
    const omni_plan_msgs::msg::PlanExecutionStatus::SharedPtr &msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->plan_execution_status_ = *msg;
  this->has_plan_execution_status_ = true;
}

void DataManager::set_fsm_state(
    const yasmin_msgs::msg::StateMachine::SharedPtr &msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->fsm_state_ = *msg;
  this->has_fsm_state_ = true;
}

omni_plan_msgs::msg::ActionInfoArray DataManager::get_actions_info() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->actions_info_;
}

omni_plan_msgs::msg::PlanExecutionStatus
DataManager::get_plan_execution_status() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->plan_execution_status_;
}

yasmin_msgs::msg::StateMachine DataManager::get_fsm_state() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->fsm_state_;
}

bool DataManager::has_actions_info() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->has_actions_info_;
}

bool DataManager::has_plan_execution_status() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->has_plan_execution_status_;
}

bool DataManager::has_fsm_state() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->has_fsm_state_;
}

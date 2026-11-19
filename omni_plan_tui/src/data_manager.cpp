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

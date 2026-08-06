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

#include <chrono>
#include <functional>

#include "omni_plan_tui/tui_node.hpp"

using namespace std::chrono_literals;
using namespace omni_plan_tui;

TuiNode::TuiNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("omni_plan_tui", options) {

  this->data_manager_ = std::make_unique<DataManager>();
  this->renderer_ = std::make_unique<TuiRenderer>();

  // Latched QoS for the action info topic (transient_local)
  auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

  this->actions_info_sub_ =
      this->create_subscription<omni_plan_msgs::msg::ActionInfoArray>(
          "/omni_plan/actions_info", latched_qos,
          std::bind(&TuiNode::actions_info_callback, this,
                    std::placeholders::_1));

  this->plan_execution_sub_ =
      this->create_subscription<omni_plan_msgs::msg::PlanExecutionStatus>(
          "/omni_plan/plan_execution", rclcpp::QoS(10).reliable(),
          std::bind(&TuiNode::plan_execution_callback, this,
                    std::placeholders::_1));

  // fsm_viewer uses a regular 10-depth publisher
  this->fsm_sub_ = this->create_subscription<yasmin_msgs::msg::StateMachine>(
      "/fsm_viewer", rclcpp::QoS(10),
      std::bind(&TuiNode::fsm_state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "OmniPlan TUI node initialized");
}

TuiNode::~TuiNode() { this->stop(); }

void TuiNode::run() {
  if (!this->renderer_->initialize()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize TUI renderer");
    return;
  }

  this->running_.store(true);

  // ~30 FPS
  this->render_timer_ =
      this->create_wall_timer(33ms, std::bind(&TuiNode::render_callback, this));

  RCLCPP_INFO(this->get_logger(), "TUI running...");
}

void TuiNode::stop() {
  this->running_.store(false);

  if (this->render_timer_) {
    this->render_timer_->cancel();
    this->render_timer_.reset();
  }

  if (this->renderer_) {
    this->renderer_->shutdown();
  }
}

void TuiNode::actions_info_callback(
    const omni_plan_msgs::msg::ActionInfoArray::SharedPtr msg) {
  if (this->data_manager_) {
    this->data_manager_->set_actions_info(msg);
  }
}

void TuiNode::plan_execution_callback(
    const omni_plan_msgs::msg::PlanExecutionStatus::SharedPtr msg) {
  if (this->data_manager_) {
    this->data_manager_->set_plan_execution_status(msg);
  }
}

void TuiNode::fsm_state_callback(
    const yasmin_msgs::msg::StateMachine::SharedPtr msg) {

  if (msg->states[0].name != "OMNI_PLANNING") {
    return;
  }

  if (this->data_manager_) {
    this->data_manager_->set_fsm_state(msg);
  }
}

void TuiNode::render_callback() {
  if (!this->running_.load() || !this->renderer_ || !this->data_manager_) {
    return;
  }

  if (!this->renderer_->handle_input(*this->data_manager_)) {
    this->stop();
    rclcpp::shutdown();
    return;
  }

  this->renderer_->render(*this->data_manager_);
}

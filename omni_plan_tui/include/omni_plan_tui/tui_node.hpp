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

#ifndef OMNI_PLAN_TUI__TUI_NODE_HPP_
#define OMNI_PLAN_TUI__TUI_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan_msgs/msg/action_info_array.hpp"
#include "omni_plan_msgs/msg/plan_execution_status.hpp"
#include "yasmin_msgs/msg/state_machine.hpp"

#include "omni_plan_tui/data_manager.hpp"
#include "omni_plan_tui/tui_renderer.hpp"

namespace omni_plan_tui {

/**
 * @class TuiNode
 * @brief ROS 2 node that drives the OmniPlan TUI.
 *
 * Subscribes to the three monitoring topics, copies incoming data into the
 * DataManager, and drives the ncurses renderer via a wall-timer at ~30 FPS.
 *
 * Topics subscribed:
 *  - `/omni_plan/actions_info`   (ActionInfoArray,        transient-local)
 *  - `/omni_plan/plan_execution` (PlanExecutionStatus,    default QoS)
 *  - `/fsm_viewer`               (yasmin_msgs/StateMachine, default QoS)
 */
class TuiNode : public rclcpp::Node {
public:
  /**
   * @brief Construct the node, create subscriptions and the render timer.
   *
   * @param options  ROS 2 node options (default: empty NodeOptions).
   */
  explicit TuiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the node, calling stop() to shut down ncurses cleanly.
   */
  ~TuiNode() override;

  /**
   * @brief Block the calling thread, spinning until stop() is called or input
   *        returns quit.
   */
  void run();

  /**
   * @brief Signal the run() loop to exit and shut down the renderer.
   */
  void stop();

  /**
   * @brief Return whether the node is currently running.
   *
   * @return true if run() is executing and has not yet been stopped.
   */
  bool is_running() const { return this->running_.load(); }

private:
  /**
   * @brief Subscription callback for the action-info latched topic.
   *
   * @param msg  Incoming ActionInfoArray message.
   */
  void actions_info_callback(
      const omni_plan_msgs::msg::ActionInfoArray::SharedPtr msg);

  /**
   * @brief Subscription callback for plan-execution status updates.
   *
   * @param msg  Incoming PlanExecutionStatus message.
   */
  void plan_execution_callback(
      const omni_plan_msgs::msg::PlanExecutionStatus::SharedPtr msg);

  /**
   * @brief Subscription callback for YASMIN state-machine snapshots.
   *
   * @param msg  Incoming StateMachine message.
   */
  void fsm_state_callback(const yasmin_msgs::msg::StateMachine::SharedPtr msg);

  /**
   * @brief Wall-timer callback that processes input and renders one frame.
   *
   * Fired at approximately 30 Hz.  Calls handle_input(); if it returns false
   * (quit requested), calls stop().  Then calls render().
   */
  void render_callback();

  /// @brief Subscription for the latched action-info topic.
  rclcpp::Subscription<omni_plan_msgs::msg::ActionInfoArray>::SharedPtr
      actions_info_sub_;
  /// @brief Subscription for plan-execution status updates.
  rclcpp::Subscription<omni_plan_msgs::msg::PlanExecutionStatus>::SharedPtr
      plan_execution_sub_;
  /// @brief Subscription for YASMIN state-machine snapshots.
  rclcpp::Subscription<yasmin_msgs::msg::StateMachine>::SharedPtr fsm_sub_;

  /// @brief Wall-timer driving the ~30 FPS render loop.
  rclcpp::TimerBase::SharedPtr render_timer_;

  /// @brief Thread-safe store for the latest monitoring data.
  std::unique_ptr<DataManager> data_manager_;
  /// @brief ncurses renderer used to draw each frame.
  std::unique_ptr<TuiRenderer> renderer_;

  /// @brief True while the run() loop is executing.
  std::atomic<bool> running_{false};
};

} // namespace omni_plan_tui

#endif // OMNI_PLAN_TUI__TUI_NODE_HPP_

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

#ifndef OMNI_PLAN_TUI__DATA_MANAGER_HPP_
#define OMNI_PLAN_TUI__DATA_MANAGER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "omni_plan_msgs/msg/action_info_array.hpp"
#include "omni_plan_msgs/msg/plan_execution_status.hpp"
#include "yasmin_msgs/msg/state_machine.hpp"

namespace omni_plan_tui {

/**
 * @class DataManager
 * @brief Thread-safe store for the latest OmniPlan monitoring data.
 *
 * All setters are intended to be called from ROS 2 subscription callbacks
 * (potentially on a different thread), while all getters are called from the
 * ncurses render timer.  A single mutex guards all three data fields.
 */
class DataManager {
public:
  /** @brief Default-construct with no data available. */
  DataManager() = default;

  // ── Setters (called from ROS callbacks) ────────────────────────

  /**
   * @brief Store the latest action-info array received from the latched topic.
   *
   * @param msg  Shared pointer to the incoming ActionInfoArray message.
   */
  void
  set_actions_info(const omni_plan_msgs::msg::ActionInfoArray::SharedPtr &msg);

  /**
   * @brief Store the latest plan-execution status snapshot.
   *
   * @param msg  Shared pointer to the incoming PlanExecutionStatus message.
   */
  void set_plan_execution_status(
      const omni_plan_msgs::msg::PlanExecutionStatus::SharedPtr &msg);

  /**
   * @brief Store the latest YASMIN state-machine snapshot.
   *
   * @param msg  Shared pointer to the incoming StateMachine message.
   */
  void set_fsm_state(const yasmin_msgs::msg::StateMachine::SharedPtr &msg);

  // ── Getters (called from render thread) ────────────────────────

  /**
   * @brief Return a copy of the latest action-info array.
   *
   * @return Copy of the stored ActionInfoArray.
   */
  omni_plan_msgs::msg::ActionInfoArray get_actions_info() const;

  /**
   * @brief Return a copy of the latest plan-execution status.
   *
   * @return Copy of the stored PlanExecutionStatus.
   */
  omni_plan_msgs::msg::PlanExecutionStatus get_plan_execution_status() const;

  /**
   * @brief Return a copy of the latest FSM state snapshot.
   *
   * @return Copy of the stored StateMachine message.
   */
  yasmin_msgs::msg::StateMachine get_fsm_state() const;

  /**
   * @brief Return whether at least one ActionInfoArray message has been stored.
   *
   * @return true if set_actions_info() has been called at least once.
   */
  bool has_actions_info() const;

  /**
   * @brief Return whether at least one PlanExecutionStatus message has been
   * stored.
   *
   * @return true if set_plan_execution_status() has been called at least once.
   */
  bool has_plan_execution_status() const;

  /**
   * @brief Return whether at least one StateMachine message has been stored.
   *
   * @return true if set_fsm_state() has been called at least once.
   */
  bool has_fsm_state() const;

private:
  /// @brief Mutex protecting all three data fields and their availability
  /// flags.
  mutable std::mutex mutex_;

  /// @brief Most recently received action info array.
  omni_plan_msgs::msg::ActionInfoArray actions_info_;
  /// @brief Most recently received plan execution status.
  omni_plan_msgs::msg::PlanExecutionStatus plan_execution_status_;
  /// @brief Most recently received YASMIN state machine snapshot.
  yasmin_msgs::msg::StateMachine fsm_state_;

  /// @brief Set to true once the first ActionInfoArray has been received.
  bool has_actions_info_ = false;
  /// @brief Set to true once the first PlanExecutionStatus has been received.
  bool has_plan_execution_status_ = false;
  /// @brief Set to true once the first StateMachine message has been received.
  bool has_fsm_state_ = false;
};

} // namespace omni_plan_tui

#endif // OMNI_PLAN_TUI__DATA_MANAGER_HPP_

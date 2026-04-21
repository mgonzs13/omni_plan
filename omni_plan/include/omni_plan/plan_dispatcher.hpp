// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#ifndef OMNI_PLAN__PLAN_DISPATCHER_HPP_
#define OMNI_PLAN__PLAN_DISPATCHER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/planning_graph.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/pddl_manager.hpp"
#include "omni_plan/utils/parameter_loader.hpp"
#include "omni_plan_msgs/msg/plan_action_status.hpp"
#include "omni_plan_msgs/msg/plan_execution_status.hpp"

namespace omni_plan {

/**
 * @class PlanDispatcher
 * @brief Abstract base class for PDDL plan dispatchers.
 * @details Defines the plugin interface for dispatching PDDL plans. Concrete
 * implementations decide the execution strategy (sequential, parallel,
 * behaviour-tree-based, etc.).  The base class provides shared infrastructure
 * for PDDL effect management, per-node status tracking, cancellation and
 * ROS-topic publishing so that derived classes only need to implement the
 * `dispatch_actions()` method.
 */
class PlanDispatcher : public utils::ParameterLoader {
public:
  /**
   * @brief Default constructor. Registers the common ROS parameters.
   */
  PlanDispatcher();

  /**
   * @brief Virtual destructor.
   */
  virtual ~PlanDispatcher() = default;

  /**
   * @brief Initialises the dispatcher with a ROS node.
   * @details Must be called once, before any call to dispatch_plan(), to
   * create the execution-status publisher and store the node for logging.
   * @param node The ROS 2 node used for publishing and logging.
   * @param pddl_manager The PDDL manager whose state is updated as effects are
   applied.
   */
  void initialize(rclcpp::Node::SharedPtr node,
                  std::shared_ptr<PddlManager> pddl_manager);

  /**
   * @brief Executes a plan by dispatching its actions.
   * @details Sets up per-node status tracking, optionally starts the
   * new-goal monitor, delegates to `dispatch_actions()` and publishes the
   * final result.
   * @param all_nodes     All nodes of the planning graph, re-indexed to
   *                      [0, N).
   * @return The overall ActionStatus once every node has finished.
   */
  pddl::ActionStatus
  dispatch_plan(const std::vector<pddl::GraphNode::Ptr> &all_nodes);

  /**
   * @brief Cancels an ongoing dispatch.
   * @details Cancels every currently running action and sets the
   * is_canceled flag so that pending nodes are skipped.
   */
  virtual void cancel_plan();

  /**
   * @brief Returns whether the dispatcher has been cancelled.
   */
  bool is_canceled() const;

protected:
  // -------------------------------------------------------------------------
  // Pure-virtual hook — implement in each concrete dispatcher
  // -------------------------------------------------------------------------

  /**
   * @brief Executes all nodes in the planning graph.
   * @details This is the core hook that each dispatcher strategy must
   * implement. It is called from `dispatch_plan()` after status tracking has
   * been initialised.
   * @param all_nodes           All graph nodes, indexed by node_num.
   * @return The overall execution result.
   */
  virtual pddl::ActionStatus
  dispatch_actions(const std::vector<pddl::GraphNode::Ptr> &all_nodes) = 0;

  // -------------------------------------------------------------------------
  // Shared infrastructure available to derived classes
  // -------------------------------------------------------------------------

  /**
   * @brief Adds an action to the dispatcher.
   * @param action The action to add.
   * @param use_cache Whether to use the cache to acquire the action if already
   * used.
   * @return The action
   */
  std::shared_ptr<pddl::Action>
  push_current_action(std::shared_ptr<pddl::Action> action,
                      bool use_cache = false);

  /**
   * @brief Removes an action from the dispatcher.
   * @param action The action to remove.
   * acquired from the cache.
   */
  void remove_current_action(std::shared_ptr<pddl::Action> action);

  /**
   * @brief Clears all actions from the dispatcher.
   */
  void clear_current_actions();

  /**
   * @brief Sets the status of a node.
   * @param node_num The node number.
   * @param status The status to set.
   */
  void set_node_status(int node_num, uint8_t status);

  /**
   * @brief Publishes the current per-node execution status.
   * @param overall The overall status code to embed in the message.
   */
  void publish_exec_status(uint8_t overall);

  /**
   * @brief Returns an idle Action instance from the pool, creating one if
   * necessary.
   */
  std::shared_ptr<pddl::Action>
  acquire_cached_action(std::shared_ptr<pddl::Action> action);

  /**
   * @brief Returns a non-primary Action instance to the pool for reuse.
   */
  void release_cached_action(std::shared_ptr<pddl::Action> action);

  /**
   * @brief Instantiates effect arguments using the action's bound parameters.
   */
  static std::vector<pddl::Effect>
  instantiate_effects(const std::vector<pddl::Effect> &effects,
                      const std::shared_ptr<pddl::Action> &action,
                      const std::vector<std::string> &params);

  /**
   * @brief Instantiates and applies a list of effects to the PDDL manager.
   * @return The applied (instantiated) effects, suitable for later rollback.
   */
  std::vector<pddl::Effect>
  apply_effects(const std::vector<pddl::Effect> &effects,
                const std::shared_ptr<pddl::Action> &action,
                const std::vector<std::string> &params);

  /**
   * @brief Reverses and removes a list of previously applied effects.
   */
  void undo_effects(const std::vector<pddl::Effect> &effects);

  /**
   * @brief Runs a single graph-node action and manages its PDDL effects.
   * @details Applies on-start and overall effects, runs the action, then
   * undoes overall effects and either commits end effects (on success) or
   * rolls back on-start effects (on failure/abort).
   * @return The ActionStatus reported by the action.
   */
  pddl::ActionStatus
  run_node_action(const pddl::GraphNode::Ptr &node,
                  const std::shared_ptr<pddl::Action> &action);

  // -------------------------------------------------------------------------
  // Protected member variables
  // -------------------------------------------------------------------------
  /// @brief Whether to cancel execution when any action aborts.
  bool cancel_on_abort_ = false;
  /// @brief Whether to cancel execution when new goals are detected.
  bool cancel_on_new_goals_ = false;

  /// @brief Mutex protecting current_actions_.
  std::mutex actions_mutex_;
  /// @brief The pluginlib class loader for Action plugins (used for cloning).
  pluginlib::ClassLoader<pddl::Action> action_state_loader_;
  /// @brief The currently running Action instances, used for cancellation.
  std::vector<std::shared_ptr<pddl::Action>> current_actions_;

  /// @brief Mutex protecting the action cache.
  std::mutex action_cache_mutex_;
  /// @brief Cache of Action instances for reuse across parallel branches.
  std::unordered_map<std::string, std::vector<std::shared_ptr<pddl::Action>>>
      action_cache_;

  /// @brief Mutex protecting PDDL manager calls.
  std::mutex pddl_manager_mutex_;
  /// @brief The PDDL manager whose state is updated as effects are applied.
  std::shared_ptr<PddlManager> pddl_manager_;

  /// @brief Mutex protecting exec_node_status_.
  std::mutex exec_node_status_mutex_;
  /// @brief Per-node execution status, indexed by node_num.
  std::vector<omni_plan_msgs::msg::PlanActionStatus> exec_node_status_;
  /// @brief Time point when the current dispatch started.
  std::chrono::steady_clock::time_point exec_start_time_;

  /// @brief ROS node for logging and publishing.
  rclcpp::Node::SharedPtr node_;
  /// @brief Publisher for live execution-status updates.
  rclcpp::Publisher<omni_plan_msgs::msg::PlanExecutionStatus>::SharedPtr
      exec_status_pub_;

  /// @brief Atomic flag indicating whether execution has been cancelled.
  std::atomic<bool> is_canceled_{false};
};

} // namespace omni_plan

#endif // OMNI_PLAN__PLAN_DISPATCHER_HPP_

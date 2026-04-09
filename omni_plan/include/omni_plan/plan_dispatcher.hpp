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
 * @brief Executes a PDDL plan by dispatching its actions in parallel.
 * @details Manages the parallel execution of plan actions according to their
 * causal dependencies as expressed in a PlanningGraph. Actions whose
 * dependencies have all succeeded are dispatched concurrently to a thread
 * pool, maximising parallelism across independent branches. The dispatcher
 * tracks per-action execution status and publishes live updates on the
 * /omni_plan/plan_execution topic.
 */
class PlanDispatcher : public utils::ParameterLoader {
public:
  /**
   * @brief Constructs a PlanDispatcher and initialises the action plugin
   * loader.
   * @param node The ROS node for logging and parameter loading.
   */
  PlanDispatcher(rclcpp::Node::SharedPtr node);

  /**
   * @brief Executes a plan by dispatching its actions in parallel.
   * @details Builds a PlanningGraph from @p plan and @p problem, then runs
   * all actions in it. Each action is executed as soon as all its
   * dependencies have succeeded. The method blocks until every action has
   * finished or the execution is cancelled.
   * @param all_nodes The list of all nodes in the planning graph.
   * @param actions_map Map from action name to the primary Action instance.
   * @param actions_plugins_map Map from action name to pluginlib class name,
   *   used to clone extra instances for parallel branches.
   * @param pddl_manager The PDDL manager; its state is updated as effects are
   *   applied and undone.
   * @return "succeeded", "canceled", or "aborted" depending on the overall
   * execution result.
   */
  pddl::ActionStatus dispatch_plan(
      const std::vector<pddl::GraphNode::Ptr> &all_nodes,
      const std::unordered_map<std::string, std::shared_ptr<pddl::Action>>
          &actions_map,
      const std::unordered_map<std::string, std::string> &actions_plugins_map,
      const std::shared_ptr<PddlManager> &pddl_manager);

  /**
   * @brief Cancels an ongoing dispatch.
   * @details Requests cancellation of every currently running action and sets
   * the internal cancelled flag so that pending nodes are skipped.
   */
  void cancel_plan();

  /**
   * @brief Returns whether the dispatcher has been cancelled.
   * @return True if cancel() has been called and not yet reset.
   */
  bool is_canceled() const;

private:
  /// @brief Whether to cancel execution when any action aborts (default:
  /// false).
  bool cancel_on_abort_ = false;
  /// @brief Whether to cancel execution when new goals are detected during
  bool cancel_on_new_goals_ = false;
  /// @brief Number of threads to use for parallel execution (default: hardware
  int execution_threads_ = 0;

  /// @brief The pluginlib class loader for Action plugins.
  pluginlib::ClassLoader<pddl::Action> action_state_loader_;
  /// @brief The currently running Action instances, used for cancellation.
  std::vector<std::shared_ptr<pddl::Action>> current_actions_;
  /// @brief Thread for monitoring new goals during execution when
  /// cancel_on_new_goals is true.
  std::thread monitor_thread_;
  /// @brief Cache of actions for reuse across parallel branches, keyed by
  /// action name.
  std::unordered_map<std::string, std::vector<std::shared_ptr<pddl::Action>>>
      action_cache_;
  /// @brief Per-node execution status, indexed by node_num. Updated in-place
  /// by worker threads and published on every change.
  std::vector<omni_plan_msgs::msg::PlanActionStatus> exec_node_status_;
  /// @brief Time point when execution started, used for computing relative
  std::chrono::steady_clock::time_point exec_start_time_;

  /// @brief Mutex for synchronising access to current_actions_.
  std::mutex actions_mutex_;
  /// @brief Mutex for synchronising access to the PDDL manager during effects
  /// application and undoing.
  std::mutex pddl_manager_mutex_;
  /// @brief Mutex for synchronising access to the action cache.
  std::mutex action_cache_mutex_;
  /// @brief Mutex for synchronising access to exec_node_status_.
  std::mutex exec_node_status_mutex_;
  /// @brief Atomic flag indicating whether execution has been cancelled.
  std::atomic<bool> is_canceled_{false};

  /// @brief Node for logging and parameter loading.
  rclcpp::Node::SharedPtr node_;
  /// @brief Publisher for execution status updates.
  rclcpp::Publisher<omni_plan_msgs::msg::PlanExecutionStatus>::SharedPtr
      exec_status_pub_;

  /**
   * @brief Returns an idle Action instance from the pool, creating one if
   *   necessary.
   */
  std::shared_ptr<pddl::Action>
  acquire_cached_action(const std::string &action_name,
                        const std::string &plugin_name);

  /**
   * @brief Returns a non-primary Action instance to the pool for reuse.
   */
  void release_cached_action(const std::string &action_name,
                             std::shared_ptr<pddl::Action> action);

  /**
   * @brief Publishes the current per-node execution status.
   * @param overall The overall status code to embed in the message.
   */
  void publish_exec_status(uint8_t overall);

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
  static std::vector<pddl::Effect>
  apply_effects(const std::vector<pddl::Effect> &effects,
                const std::shared_ptr<pddl::Action> &action,
                const std::vector<std::string> &params,
                const std::shared_ptr<PddlManager> &pddl_manager);

  /**
   * @brief Reverses and removes a list of previously applied effects.
   */
  static void undo_effects(const std::vector<pddl::Effect> &effects,
                           const std::shared_ptr<PddlManager> &pddl_manager);

  /**
   * @brief Runs a single graph-node action and manages its PDDL effects.
   * @return The ActionStatus reported by the action.
   */
  pddl::ActionStatus
  run_node_action(const pddl::GraphNode::Ptr &node,
                  const std::shared_ptr<pddl::Action> &action,
                  const std::shared_ptr<PddlManager> &pddl_manager);

  /**
   * @brief Executes all nodes in the graph using a bounded thread pool.
   * @return "succeeded", "canceled", or "aborted" depending on the overall
   * execution result.
   */
  pddl::ActionStatus execute_branches(
      const std::vector<pddl::GraphNode::Ptr> &all_nodes,
      std::shared_ptr<PddlManager> pddl_manager,
      const std::unordered_map<std::string, std::shared_ptr<pddl::Action>>
          &actions_map,
      std::unordered_map<std::string, std::string> actions_plugins_map);
};

} // namespace omni_plan

#endif // OMNI_PLAN__PLAN_DISPATCHER_HPP_
